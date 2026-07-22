# Builder image for generating px4-ros2-interface-lib Debian packages with bloom.
# The ROS 2 distribution is selectable so packages can be produced for every
# distribution on the build farm (humble, jazzy, kilted, rolling, lyrical, ...).
#
# px4_ros2_cpp build-depends on px4_msgs, which is not yet on the ROS build farm,
# so the in-container build script (build_deb_container.sh) builds and installs
# px4_msgs from source first, then px4_ros2_cpp, then px4_ros2_py, in that order.
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-core

# Redeclare after FROM so it is available in the build stage.
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

# The tooling required to resolve dependencies and generate/build the Debian
# packages. build-essential provides the C/C++ compiler; git is needed to fetch
# the px4_msgs dependency source; python3-empy backs the px4_ros2_cpp message
# header codegen.
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  python3-bloom \
  python3-rosdep \
  python3-empy \
  build-essential \
  cmake \
  git \
  fakeroot \
  debhelper \
  dh-python \
  && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

WORKDIR /work

COPY container/build_deb_container.sh /usr/local/bin/build.sh
RUN chmod +x /usr/local/bin/build.sh
