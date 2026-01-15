import px4_ros2

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition


class TestNode(Node):
    def __init__(self) -> None:
        super().__init__("test_node")

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Create a subscriber
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v" + str(VehicleLocalPosition.MESSAGE_VERSION),
            self.vehicle_local_position_callback,
            qos_profile,
        )

        self.counter = 0

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.counter += 1
        if self.counter % 50 == 0:
            print(
                f"Local position update: {vehicle_local_position.x}, {vehicle_local_position.y}, {vehicle_local_position.z}"
            )


class MyMode(px4_ros2.components.ModeBase):
    def __init__(self, node, mode_name="My Test Mode"):
        super().__init__(node=node, mode_name=mode_name)

        self.setpoint = px4_ros2.control.MulticopterGotoSetpointType(self)
        self.local_position = px4_ros2.odometry.LocalPosition(self)
        self.position_reached = False

    def on_activate(self):
        print("Mode activated")
        self.position_reached = False

    def on_deactivate(self):
        print("Mode deactivated")

    def update_setpoint(self, dt_s: float):
        target_position = (30, 30, -10)
        self.setpoint.update(target_position, heading=px4_ros2.geometry.deg_to_rad(30))
        if not self.position_reached and np.linalg.norm(self.local_position.position_ned() - target_position) < 1:
            self.position_reached = True
            print(f"Position reached: {self.local_position.position_ned()}")
            self.completed()


class MyModeExecutor(px4_ros2.components.ModeExecutorBase):
    def __init__(self, mode):
        super().__init__(mode)
        self.mode = mode

    def on_activate(self):
        print("ModeExecutor activated")
        self.takeoff(self.takeoff_completed)

    def takeoff_completed(self, result):
        if result == px4_ros2.components.Result.success:
            self.schedule_mode(self.mode.id(), self.own_mode_completed)

    def own_mode_completed(self, result):
        if result == px4_ros2.components.Result.success:
            self.rtl(self.rtl_completed)

    def rtl_completed(self, result):
        print(f"Completed: {result}")

    def on_deactivate(self, reason):
        print(f"ModeExecutor deactivated: {reason}")


def main(args=None):
    """Example demonstrating usage of both px4_ros2 and rclpy.

    Due to underlying incompatibilities, two ROS nodes need to be created.

    Note:
        If there is data shared (and modified) between the two nodes
        (e.g. in callbacks), a mutex like threading.Lock must be used.
    """

    # Init px4_ros2
    node = px4_ros2.Node("rlcpp_node", debug_output=True)
    mode = MyMode(node)
    mode_executor = MyModeExecutor(mode)
    assert mode_executor.register()
    node.spin_non_blocking()

    # Init rclpy
    rclpy.init(args=args)
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
