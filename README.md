# PX4 ROS 2 Interface Library

Library to interface with PX4 from a companion computer using ROS 2.
It provides some tooling used to write external modes that are dynamically registered with PX4 and behave the same way as internal ones.
A mode can send different types of setpoints, ranging from high-level navigation tasks all the way down to direct actuator controls.

Documentation:
- High-level: https://docs.px4.io/main/en/ros2/px4_ros2_interface_lib.html
- API: https://auterion.github.io/px4-ros2-interface-lib/topics.html

## Compatibility with PX4
The library interacts with PX4 by using its uORB messages, and thus requires a matching set of message definitions on the ROS 2 side.
To ensure compatibility, you must either:

1. Use latest `main` on the PX4 and px4_ros2/px4_msgs sides, which should define matching messages
1. (Experimental) Run the PX4 [message translation node](https://github.com/PX4/PX4-Autopilot/tree/main/msg/translation_node), which dynamically monitors and translates PX4 messages when different message version are used within the same ROS 2 domain

### Option 1: Match Messages

The library checks for message compatibility on startup when registering a mode.
`ALL_PX4_ROS2_MESSAGES` defines the set of checked messages. If you use other messages, you can check them using:
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

If you intend to run the message translation node to use mismatching message versions in PX4 and px4_ros2/px4_msgs, then you must manually disable the message compatibility check that runs when registering a mode.
This can be done the following way:

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
There are code examples under [examples/cpp/modes](examples/cpp/modes).

## Development
For development, install the pre-commit scripts:
```shell
pre-commit install
```

### CI
CI runs a number of checks which can be executed locally with the following commands.
Make sure you have the ROS workspace sourced.

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
