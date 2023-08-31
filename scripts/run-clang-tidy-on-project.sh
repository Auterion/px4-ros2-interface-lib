#! /bin/bash
set -e

THIS_DIR="$(dirname $(readlink -f $0))"
ROOT_DIR="$(dirname "$THIS_DIR")"
# Find all ROS package names in this repo
PACKAGES=()
for package_xml in $(find "$ROOT_DIR" -name package.xml); do
  package_name=$(cat "$package_xml" | sed -n 's/.*<name>\(.*\)<\/name>.*/\1/p')
  PACKAGES+=($package_name)
done

if [ "$1" == "--help" -o "$1" == "-h" ]; then
  echo "Usage: $0 [<ros2-build-dir>]"
  exit 0
fi

if [ $# -gt 0 ]; then # Optionally set the build dir from CLI
  BUILD_DIR="$(readlink -f $1)"
else
  PACKAGE_INSTALL_DIR="$(ros2 pkg prefix ${PACKAGES[0]})"
  BUILD_DIR="$PACKAGE_INSTALL_DIR/../../build"
fi

for PACKAGE in "${PACKAGES[@]}"; do
  pushd "$BUILD_DIR/$PACKAGE" &>/dev/null
  # For some reason we explicitly have to specify c++17 in ROS CI
  "$THIS_DIR"/run-clang-tidy.py -p . -use-color -header-filter="$ROOT_DIR/.*" -quiet -extra-arg=-std=c++17
  popd &>/dev/null
done