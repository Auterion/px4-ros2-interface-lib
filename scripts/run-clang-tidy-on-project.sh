#! /bin/bash
set -e

THIS_DIR="$(dirname $(readlink -f $0))"
ROOT_DIR="$(dirname "$THIS_DIR")"
# Find all ROS package names in this repo
PACKAGES=()
for package_xml in $(find "$ROOT_DIR" -name package.xml); do
  package_name=$(cat "$package_xml" | sed -n 's/.*<name>\(.*\)<\/name>.*/\1/p')
  package_dir="$(dirname "$package_xml")"
  # Only include C++ packages
  if [ -f "$package_dir/CMakeLists.txt" ]; then
    PACKAGES+=($package_name)
  else
    echo "Skipping package $package_name in $package_dir (not C++)"
  fi
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
  echo "Checking $BUILD_DIR/$PACKAGE"
  # For some reason we explicitly have to specify c++17 in ROS CI.
  # Also suppresses the 'optimization flag '-fno-fat-lto-objects' is not supported' error
  "$THIS_DIR"/run-clang-tidy.py -p . -use-color -header-filter="$ROOT_DIR/.*" -quiet \
    -extra-arg=-std=c++17 -extra-arg="-Wno-unknown-warning-option" -extra-arg="-Wno-ignored-optimization-argument"
  popd &>/dev/null
done