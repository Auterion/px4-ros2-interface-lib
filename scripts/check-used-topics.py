#!/usr/bin/env python3
""" Script to check all used topics in the code against a list used for message compatibility check """

import argparse
import os
import re

from typing import Optional

ignored_topics = ['message_format_request', 'message_format_response']

configs = [
    # Tuples of (topics_list_file, topic define, source_dir list)
    ('px4_ros2_cpp/include/px4_ros2/components/message_compatibility_check.hpp', 'ALL_PX4_ROS2_MESSAGES', [
        'px4_ros2_cpp/src', 'px4_ros2_cpp/include'
    ]),
]

project_root_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..')


def extract_topics_from_file(filename: str, extract_start_after: Optional[str] = None,
                             extract_end_before: Optional[str] = None) -> list[str]:
    with open(filename) as file:
        if extract_start_after is not None:
            for line in file:
                if re.search(extract_start_after, line):
                    break

        ret = []
        for line in file:
            m = re.search(r'"fmu/(in|out)/([^"]+)', line)
            if m:
                ret.append(m.group(2))

            if extract_end_before is not None and re.search(extract_end_before, line):
                break

        return ret


def check(verbose: bool):
    for topics_list_file, define, source_list in configs:
        topics_list_file = os.path.join(project_root_dir, topics_list_file)

        checked_topics = extract_topics_from_file(topics_list_file, define, r'^\s*$')
        if verbose:
            print(f'checked topics: {checked_topics}')
        assert len(checked_topics) > 0

        all_used_topics = []
        for source_dir in source_list:
            source_dir = os.path.join(project_root_dir, source_dir)
            # Iterate recursively
            for subdir, dirs, files in os.walk(source_dir):
                for file in files:
                    if file.endswith('.hpp') or file.endswith('.cpp'):
                        file_name = os.path.join(subdir, file)

                        if os.path.normpath(file_name) == os.path.normpath(topics_list_file):
                            continue

                        if verbose:
                            print(f'extracting from file: {file_name}')
                        all_used_topics.extend(extract_topics_from_file(file_name))

        all_used_topics = set(all_used_topics)  # remove duplicates
        if verbose:
            print(f'used topics: {all_used_topics}')

        not_found_topics = []
        for used_topic in all_used_topics:
            if used_topic not in checked_topics and used_topic not in ignored_topics:
                not_found_topics.append(used_topic)
        if len(not_found_topics) > 0:
            raise RuntimeError(f'Topic(s) {not_found_topics} are not in the list of checked topics ({define}).\n'
                               f'Add the topic to the file {topics_list_file}')

        not_used_topics = []
        for checked_topic in checked_topics:
            if checked_topic not in all_used_topics and checked_topic not in ignored_topics:
                not_used_topics.append(checked_topic)
        if len(not_used_topics) > 0:
            raise RuntimeError(f'Topic(s) {not_used_topics} are in the list of checked topics ({define}), but not '
                               f'used in code.\n'
                               f'Remove the topic from the file {topics_list_file}')


def main():
    parser = argparse.ArgumentParser(description='Check used PX4 topics in source')

    parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                        help='Verbose Output')

    args = parser.parse_args()
    check(args.verbose)


if __name__ == '__main__':
    main()
