# px4_ros2_sdk (experimental)

A ROS 2-native flight SDK layered strictly on top of the `px4_ros2_cpp` device-driver core (it adds no new `/fmu` endpoints beyond `px4_ros2_cpp`). It exposes PX4 flight capabilities as an idiomatic ROS 2 graph and provides a thin ergonomic commander over that graph, following the design in [doc/API_EXPANSION_RFC.md](../doc/API_EXPANSION_RFC.md).

This package is the P0 slice of that RFC: the bringup and the non-streaming "safe core".

- `FlightServer`: a managed-lifecycle node. `on_configure` runs the FMU discovery (fails closed if absent); `on_activate` starts an `~/arm` service and a `~/takeoff` action server, driven through `px4_ros2_cpp`'s `VehicleCommandSender`.
- `SimpleFlight`: a thin action-plus-service client facade (the "simple commander" tier) for scripts.
- `sdk::Result`: the SDK result vocabulary, reporting the vehicle acknowledgement (not physical completion).

The streaming setpoint interfaces, more maneuver actions, the telemetry hub, pluginlib behaviors and the Python facade are proposed in the RFC and land in later phases.

## Example (ergonomic commander)

```cpp
#include <px4_ros2_sdk/simple_flight.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("my_app");

  // Talks to a FlightServer running in the given namespace.
  px4_ros2::sdk::SimpleFlight flight(node, "/vehicle_1");
  if (flight.arm() == px4_ros2::sdk::Result::Success) {
    flight.takeoff(/* altitude_amsl_m = */ 30.f);
  }

  rclcpp::shutdown();
  return 0;
}
```

Run the server as a lifecycle node (transition it with a `lifecycle_manager` or the `ros2 lifecycle` CLI):

```sh
ros2 run px4_ros2_sdk flight_server_node --ros-args -r __ns:=/vehicle_1
```

## Status

Experimental. The API is expected to change as the RFC phases land. It is not part of the stable `px4_ros2_cpp` ABI.

Contributed by [RIIS, LLC](https://www.riis.com).
