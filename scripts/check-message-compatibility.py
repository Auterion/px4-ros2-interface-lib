#!/usr/bin/env python3
""" Check message compatibility between two repositories containing a msg/ directory of .msg message definitions """

from __future__ import annotations  # for Python 3.8 compatibility
import os
import sys
import difflib
import re
import argparse

from typing import Optional

TOPIC_LIST_FILE = 'px4_ros2_cpp/include/px4_ros2/components/message_compatibility_check.hpp'
MESSAGES_DEFINE = 'ALL_PX4_ROS2_MESSAGES'


def message_fields_str_for_message_hash(topic_type: str, msgs_dir: str) -> str:
    """
    Reads the .msg file corresponding to the given topic type, extracts field definitions,
    and recursively processes nested types to generate a string representation of all fields.
    """
    filename = find_file_in_subfolders(f"{msgs_dir}/", f"{topic_type}.msg")
    try:
        with open(filename, 'r') as file:
            text = file.read()
    except IOError:
        print(f"Failed to open {filename}")
        return ""

    fields_str = ""

    # Regular expression to match field types from .msg definitions
    msg_field_type_regex = re.compile(
        r"(?:^|\n)\s*([a-zA-Z0-9_/]+)(\[[^\]]*\])?\s+(\w+)[ \t]*(=)?"
    )

    # Set of basic types
    basic_types = {
        "bool", "byte", "char", "float32", "float64",
        "int8", "uint8", "int16", "uint16", "int32",
        "uint32", "int64", "uint64", "string", "wstring"
    }

    # Iterate over all matches in the text
    for match in msg_field_type_regex.finditer(text):
        type_, array, field_name, constant = match.groups()

        if constant == "=":
            continue

        fields_str += f"{type_}{array} {field_name}\n"

        if type_ not in basic_types:
            if '/' not in type_:
                # Recursive call to handle nested types
                fields_str += message_fields_str_for_message_hash(type_, msgs_dir)
            else:
                raise ValueError(f"Field {filename} contains namespace {type_}")

    return fields_str


def hash32_fnv1a_const(s: str) -> int:
    """Computes the 32-bit FNV-1a hash of a given string"""
    kVal32Const = 0x811c9dc5
    kPrime32Const = 0x1000193
    hash_value = kVal32Const
    for c in s:
        hash_value ^= ord(c)
        hash_value *= kPrime32Const
        hash_value &= 0xFFFFFFFF
    return hash_value


def message_hash(topic_type: str, msgs_dir: str) -> int:
    """Generate a hash from a message definition file"""
    message_fields_str = message_fields_str_for_message_hash(topic_type, msgs_dir)
    return hash32_fnv1a_const(message_fields_str)


def snake_to_pascal(name: str) -> str:
    """Convert snake_case to PascalCase"""
    return f'{name.replace("_", " ").title().replace(" ", "")}'


def extract_message_type_from_file(filename: str, extract_start_after: Optional[str] = None,
                                   extract_end_before: Optional[str] = None) -> list[str]:
    """Extract message type names from a given file"""
    with open(filename) as file:
        if extract_start_after is not None:
            for line in file:
                if re.search(extract_start_after, line):
                    break

        message_types = set()
        for line in file:
            m = re.search(r'"fmu/(in|out)/([^"]+)"(?:, "([^"]+)")?', line)
            if m:
                if m.group(3):
                    # Use the second element directly if available
                    message_types.add(m.group(3))
                else:
                    # Convert to PascalCase if no second element is present
                    message_types.add(snake_to_pascal(m.group(2)))

            if extract_end_before is not None and re.search(extract_end_before, line):
                break

    return list(message_types)


def compare_files(file1: str, file2: str):
    """Compare two files and print their differences. """
    with open(file1, 'r') as f1, open(file2, 'r') as f2:
        diff = list(difflib.unified_diff(f1.readlines(), f2.readlines(), fromfile=file1, tofile=file2))
        if diff:
            print(f"Mismatch found between {file1} and {file2}: ")
            print(''.join(diff), end='\n\n')
            return False
    return True


def find_file_in_subfolders(root_dir, filename):
    if filename in os.listdir(root_dir):
        return os.path.join(root_dir, filename)
    for dirpath, _, filenames in os.walk(root_dir):
        if filename in filenames:
            return os.path.join(dirpath, filename)
    return None


def main(repo1: str, repo2: str, verbose: bool = False):
    if not os.path.isdir(repo1) or not os.path.isdir(repo2):
        print("Both arguments must be directories.")
        sys.exit(1)

    # Retrieve list of message types to check
    messages_types = sorted(extract_message_type_from_file(
        os.path.join(os.path.dirname(__file__), '..', TOPIC_LIST_FILE),
        MESSAGES_DEFINE,
        r'^\s*$')
    )

    if verbose:
        print("Checking the following message files:", end='\n\n')
        for msg_type in messages_types:
            print(f" - {msg_type}.msg")
        print()

    # Find mismatches
    incompatible_types = []
    for msg_type in messages_types:
        if message_hash(msg_type, repo1) != message_hash(msg_type, repo2):
            incompatible_types.append(msg_type)

    # Print result
    if not incompatible_types:
        print("OK! Messages are compatible.")
        sys.exit(0)
    else:
        if verbose:
            for msg_type in incompatible_types:
                file1 = find_file_in_subfolders(os.path.join(repo1, 'msg'), f'{msg_type}.msg')
                file2 = find_file_in_subfolders(os.path.join(repo2, 'msg'), f'{msg_type}.msg')
                compare_files(file1, file2)
            print("Note: The printed diff includes all content differences. "
                  "The computed check is less sensitive to formatting and comments.", end='\n\n')
        print("FAILED! Some files differ:")
        for msg_type in incompatible_types:
            print(f" - {msg_type}.msg")
        sys.exit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Check message compatibility between two repositories \
                                                  using the set of checked messages ALL_PX4_ROS2_MESSAGES.")
    parser.add_argument('repo1', help="path to the first repo containing a msg/ directory \
                                       (e.g /path/to/px4_msgs/)")
    parser.add_argument('repo2', help="path to the second repo containing a msg/ directory \
                                       (e.g /path/to/PX4-Autopilot/)")
    parser.add_argument('-v', '--verbose', dest='verbose', action='store_true', help='verbose output')
    args = parser.parse_args()

    main(args.repo1, args.repo2, args.verbose)
