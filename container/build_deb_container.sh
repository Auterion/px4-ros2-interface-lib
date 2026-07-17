#!/bin/bash
set -euo pipefail

# This script runs INSIDE the px4-ros2-interface-lib builder container.
# Assumptions:
#   /src         -> px4-ros2-interface-lib source tree (mounted from host)
#   /artifacts   -> output dir (mounted from host, .deb files land here)
#   ROS_DISTRO   -> ROS 2 distribution to target (provided by the image)
#   PX4_MSGS_REF -> px4_msgs git ref to build the dependency from (default: main)
#
# px4_ros2_cpp build-depends on px4_msgs and px4_ros2_py build-depends on
# px4_ros2_cpp. px4_msgs is not on the ROS build farm yet, so rosdep cannot
# resolve it. This script therefore builds each dependency into a .deb and
# installs it before building the next package, in strict order:
#   px4_msgs  ->  px4_ros2_cpp  ->  px4_ros2_py

: "${ROS_DISTRO:?ROS_DISTRO must be set (provided by the builder image)}"

# Debian packaging is Debian/Ubuntu specific. The OS name defaults to ubuntu and
# the OS version is auto-detected by bloom from the base image, so each ROS 2
# distribution is packaged for its matching Ubuntu release (jammy for humble,
# noble for jazzy/kilted/rolling, ...).
OS_NAME="${OS_NAME:-ubuntu}"
# The px4_msgs source ref providing the matching message set. Pin per release
# line (for example a release/1.17 branch or a v1.17.0 tag) via the environment.
PX4_MSGS_REF="${PX4_MSGS_REF:-main}"

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Build a single package directory into a .deb, stash the artifact, and install
# it so the next package in the chain can resolve it.
build_deb() {
  local pkg_dir="$1"
  local parent
  parent="$(dirname "${pkg_dir}")"
  echo "[*] Building $(basename "${pkg_dir}") for ROS 2 ${ROS_DISTRO} (${OS_NAME})"

  (
    cd "${pkg_dir}"
    rm -rf debian obj-* .obj-* CMakeCache.txt CMakeFiles build install log || true
    # Resolve everything rosdep can; skip the local keys that are not on the
    # build farm and are supplied as freshly built .deb dependencies instead.
    rosdep install --from-paths . --ignore-src -y --rosdistro "${ROS_DISTRO}" \
      --skip-keys "px4_msgs px4_ros2_cpp"
    bloom-generate rosdebian --os-name "${OS_NAME}" --ros-distro "${ROS_DISTRO}"
    # Skip the test suite while packaging; the linters/tests run in the build CI.
    DEB_BUILD_OPTIONS=nocheck fakeroot debian/rules binary
  )

  # bloom writes the .deb to the parent of the package directory.
  apt-get install -y "${parent}"/ros-"${ROS_DISTRO}"-*.deb
  mkdir -p /artifacts
  cp "${parent}"/*.deb /artifacts/ 2>/dev/null || true
  cp "${parent}"/*.ddeb /artifacts/ 2>/dev/null || true
  rm -f "${parent}"/*.deb "${parent}"/*.ddeb || true
}

echo "[*] Fetching px4_msgs (${PX4_MSGS_REF}) to satisfy the px4_ros2_cpp dependency"
rm -rf /work/px4_msgs
git clone https://github.com/PX4/px4_msgs.git /work/px4_msgs
git -C /work/px4_msgs checkout "${PX4_MSGS_REF}"

build_deb /work/px4_msgs
build_deb /src/px4_ros2_cpp
build_deb /src/px4_ros2_py

echo "[*] Done. Artifacts in /artifacts:"
ls -lh /artifacts/
