#!/bin/bash
set -euo pipefail

# Build px4-ros2-interface-lib Debian packages locally using the bundled builder
# container. px4_ros2_cpp and px4_ros2_py are built in dependency order, with the
# px4_msgs dependency built from source inside the container (it is not on the
# ROS build farm yet).
#
# Usage:
#   ./scripts/build_deb_host.sh                 # humble (default)
#   ROS_DISTRO=jazzy ./scripts/build_deb_host.sh
#   ROS_DISTRO=jazzy PX4_MSGS_REF=release/1.17 ./scripts/build_deb_host.sh
#
# The resulting *.deb files are written to ./out

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( realpath "${SCRIPT_DIR}/.." )"

SRC_DIR="${REPO_ROOT}"
OUT_DIR="${REPO_ROOT}/out"

ROS_DISTRO="${ROS_DISTRO:-humble}"
PX4_MSGS_REF="${PX4_MSGS_REF:-main}"
IMAGE="px4-ros2-interface-lib-builder:${ROS_DISTRO}"

mkdir -p "${OUT_DIR}"

echo "[*] Building px4-ros2-interface-lib .deb packages"
echo "[*] ROS distro:   ${ROS_DISTRO}"
echo "[*] px4_msgs ref: ${PX4_MSGS_REF}"
echo "[*] Source:       ${SRC_DIR}"
echo "[*] Artifacts:    ${OUT_DIR}"

echo "[*] Building builder image ${IMAGE}"
docker build \
    --build-arg "ROS_DISTRO=${ROS_DISTRO}" \
    -t "${IMAGE}" \
    -f "${REPO_ROOT}/container/Dockerfile" \
    "${REPO_ROOT}"

echo "[*] Running the build in a container"
docker run --rm \
    -e "PX4_MSGS_REF=${PX4_MSGS_REF}" \
    -v "${SRC_DIR}":/src \
    -v "${OUT_DIR}":/artifacts \
    "${IMAGE}" \
    /usr/local/bin/build.sh

echo "[*] Done. Check ${OUT_DIR} for .deb files"
