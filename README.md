# PX4 ROS 2 Interface Library

<!-- status -->
[![Build package](https://github.com/Auterion/px4-ros2-interface-lib/actions/workflows/build.yml/badge.svg?branch=main)](https://github.com/Auterion/px4-ros2-interface-lib/actions/workflows/build.yml)
[![Lint](https://github.com/Auterion/px4-ros2-interface-lib/actions/workflows/lint.yml/badge.svg?branch=main)](https://github.com/Auterion/px4-ros2-interface-lib/actions/workflows/lint.yml)
[![License: BSD-3-Clause](https://img.shields.io/github/license/Auterion/px4-ros2-interface-lib?color=brightgreen)](LICENSE)
<!-- ecosystem -->
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy%20%7C%20Kilted%20%7C%20Rolling%20%7C%20Lyrical-22314E?logo=ros&logoColor=white)](https://github.com/Auterion/px4-ros2-interface-lib#compatibility-matrix)
[![PX4 release line](https://img.shields.io/badge/PX4-1.17-0091EA?logo=drone&logoColor=white)](https://github.com/Auterion/px4-ros2-interface-lib#versioning)

Library to interface with PX4 from a companion computer using ROS 2. It provides some tooling used to write external modes that are dynamically registered with PX4 and behave the same way as internal ones. A mode can send different types of setpoints, ranging from high-level navigation tasks all the way down to direct actuator controls.

The library is written in C++, with Python bindings provided as well (not yet complete). It ships two ament packages: `px4_ros2_cpp` (the C++ library) and `px4_ros2_py` (the pybind11 Python bindings that wrap it).

Documentation:
- High-level: https://docs.px4.io/main/en/ros2/px4_ros2_interface_lib.html
- API:
  - C++: https://auterion.github.io/px4-ros2-interface-lib/topics.html
  - Python: https://auterion.github.io/px4-ros2-interface-lib/python/px4_ros2_py.html

## Installation

### From source (recommended today)

Clone the branch that matches your PX4 version (see the [compatibility matrix](https://github.com/Auterion/px4-ros2-interface-lib#compatibility-matrix)) into a colcon workspace alongside `px4_msgs`, then build. `px4_ros2_cpp` hard-depends on `px4_msgs` at build and runtime, and `px4_ros2_py` depends on `px4_ros2_cpp`, so the two packages must build after `px4_msgs`.

```sh
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/Auterion/px4-ros2-interface-lib.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash      # or humble / kilted / rolling
colcon build
```

To build only the C++ library (for example on platforms where the Python bindings are not needed), select the package explicitly:

```sh
colcon build --packages-up-to px4_ros2_cpp
```

### From a binary (apt), forthcoming

Binary Debian packages via `apt install ros-${ROS_DISTRO}-px4-ros2-cpp` are not yet available. Publishing to the ROS build farm is gated on `px4_msgs` (a build dependency of `px4_ros2_cpp`) first reaching the farm through [ros/rosdistro](https://github.com/ros/rosdistro); until then the packages are source-only. The release order is strictly `px4_msgs` -> `px4_ros2_cpp` -> `px4_ros2_py`. See [Versioning](https://github.com/Auterion/px4-ros2-interface-lib#versioning) and the release workflow for the current status.

## Versioning

Versions follow the **PX4 release line** and are kept in step with [`px4_msgs`](https://github.com/PX4/px4_msgs) (currently `1.17.0`), rather than an independent library semver. The in-tree `package.xml` version is authoritative and is what `bloom` reads when cutting a release.

Development happens on `main`, which tracks PX4 `main`. Each PX4 stable line has a `release/X.Y` branch (for example `release/1.17`, `release/1.16`), so you pick the branch that matches the PX4 firmware you fly. `bloom` only accepts increasing versions per ROS 2 distribution, so a later patch on an older line is released to the distributions that track that line and never overrides a newer release already published for another line.

## Compatibility matrix

The library talks to PX4 over its uORB messages, so a checkout is tied to a **PX4 release line** through the matching `px4_msgs` definitions. Pick the branch that matches your PX4 version, then build it on whichever maintained ROS 2 distribution you run.

### PX4 release line

| PX4 release             | px4_ros2 (this library) branch                                                                | px4_msgs branch                                                     |
|-------------------------|-----------------------------------------------------------------------------------------------|--------------------------------------------------------------------|
| `main` (in development) | [`main`](https://github.com/Auterion/px4-ros2-interface-lib)                                   | [`main`](https://github.com/PX4/px4_msgs)                          |
| v1.17                   | [`release/1.17`](https://github.com/Auterion/px4-ros2-interface-lib/tree/release/1.17)         | [`release/1.17`](https://github.com/PX4/px4_msgs/tree/release/1.17) |
| v1.16                   | [`release/1.16`](https://github.com/Auterion/px4-ros2-interface-lib/tree/release/1.16)         | [`release/1.16`](https://github.com/PX4/px4_msgs/tree/release/1.16) |

Additional `release/X.Y` branches follow each PX4 line as it is cut (for example `release/1.18` for the upcoming line). If PX4, `px4_msgs`, and this library do not come from the matching branches, the message sets may differ; see [Compatibility with PX4](https://github.com/Auterion/px4-ros2-interface-lib#compatibility-with-px4) for how the runtime check and the message translation node handle that.

### ROS 2 distribution to Ubuntu

The ROS 2 distribution fixes the Ubuntu version ([REP 2000](https://www.ros.org/reps/rep-2000.html)) and is independent of the PX4 line above. CI builds `main` against every maintained distribution:

| ROS 2 distribution | Ubuntu           | Type                | Built in CI                                               |
|--------------------|------------------|---------------------|-----------------------------------------------------------|
| Humble             | 22.04 (Jammy)    | LTS, until 2027     | yes, native runner                                        |
| Jazzy              | 24.04 (Noble)    | LTS, until 2029     | yes, native runner                                        |
| Kilted             | 24.04 (Noble)    | non-LTS, until 2026 | yes, native runner                                        |
| Rolling            | 24.04 (Noble)    | development         | yes, native runner                                        |
| Lyrical            | 26.04 (Resolute) | LTS                 | yes, in the `ros:lyrical` container (no 26.04 runner yet) |

Windows (Jazzy) and macOS (Jazzy, via [RoboStack](https://robostack.github.io/)) are covered by CI as well. On those platforms CI may build only `px4_ros2_cpp` where the pybind11 bindings or the `px4_msgs` codegen do not build.

## Compatibility with PX4
The library interacts with PX4 by using its uORB messages, and thus requires a matching set of message definitions on the ROS 2 side. To ensure compatibility, you must either:

1. Use latest `main` on the PX4 and px4_ros2/px4_msgs sides, which should define matching messages
1. (Experimental) Run the PX4 [message translation node](https://github.com/PX4/PX4-Autopilot/tree/main/msg/translation_node), which dynamically monitors and translates PX4 messages when different message version are used within the same ROS 2 domain
1. Use the `release/<version>` branches matching the PX4 releases, as listed in the [compatibility matrix](https://github.com/Auterion/px4-ros2-interface-lib#compatibility-matrix) above (for example [`release/1.17`](https://github.com/Auterion/px4-ros2-interface-lib/tree/release/1.17))

### Option 1: Match Messages

The library checks for message compatibility on startup when registering a mode. `ALL_PX4_ROS2_MESSAGES` defines the set of checked messages. If you use other messages, you can check them using:
```cpp
if (!px4_ros2::messageCompatibilityCheck(node, {{"fmu/in/vehicle_rates_setpoint"}})) {
  throw std::runtime_error("Messages incompatible");
}
```

To manually verify that two local versions of PX4 and px4_msgs have matching message sets, you can use the following script:

```sh
./scripts/check-message-compatibility.py -v path/to/px4_msgs/ path/to/PX4-Autopilot/
```

### Option 2: Translate Messages

If you intend to run the message translation node to use mismatching message versions in PX4 and px4_ros2/px4_msgs, then you must manually disable the message compatibility check that runs when registering a mode. This can be done the following way:

```c++
class CustomMode : public px4_ros2::ModeBase
{
public:
  CustomMode(rclcpp::Node & node)
  : ModeBase(node, "node_name")
  {
    setSkipMessageCompatibilityCheck();  // Disables compatibility check
    ...
  }
  ...
};
```

## Examples
There are code examples under [examples/cpp](examples/cpp) for C++ and [examples/python](examples/python) for Python.

## Development
For development, install the pre-commit scripts:
```shell
pre-commit install
```

### CI
CI runs a number of checks which can be executed locally with the following commands. Make sure you have the ROS workspace sourced.

#### clang-tidy
```shell
./scripts/run-clang-tidy-on-project.sh
```

#### Unit tests
You can either run the unit tests through colcon:
```shell
colcon test --packages-select px4_ros2_cpp --ctest-args -R unit_tests
colcon test-result --verbose
```
Or directly from the build directory, which allows to filter by individual tests:
```shell
./build/px4_ros2_cpp/px4_ros2_cpp_unit_tests --gtest_filter='xy*'
```

#### Linters (code formatting etc)
These run automatically when committing code. To manually run them, use:
```shell
pre-commit run -a
```
