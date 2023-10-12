# PX4 ROS 2 SDK

Library to control PX4 from a companion computer using ROS 2.
It allows to write external modes that are dynamically registered with PX4 and behave the same way as internal ones.
A mode can send different types of setpoints, ranging from high-level navigation tasks all the way down to direct actuator controls.

## Compatibility with PX4
The library interacts with PX4 by using its uORB messages, and thus requires a matching set of message definitions on the ROS 2 side.
Compatibility is only guaranteed if using latest `main` on the PX4 and SDK/px4_msgs side. This might change in the future.

The SDK checks for message compatibility on startup when registering a mode.
`ALL_PX4_SDK_MESSAGES` defines the set of checked messages. If you use other messages, you can check them using:
```c++
if (!px4_sdk::messageCompatibilityCheck(node, {{"/fmu/in/vehicle_rates_setpoint"}})) {
  throw std::runtime_error("Messages incompatible");
}
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
colcon test --packages-select px4_sdk_cpp --ctest-args -R unit_tests
colcon test-result --verbose
```
Or directly from the build directory, which allows to filter by individual tests:
```shell
./build/px4_sdk_cpp/px4_sdk_cpp_unit_tests --gtest_filter='xy*'
```

#### Linters (code formatting etc)
These run automatically when committing code. To manually run them, use:
```shell
pre-commit run -a
```
