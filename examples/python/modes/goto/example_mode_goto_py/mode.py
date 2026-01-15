import px4_ros2


class MyMode(px4_ros2.components.ModeBase):
    def __init__(self, node, mode_name="My Mode"):
        super().__init__(node=node, mode_name=mode_name)

        self.setpoint = px4_ros2.control.MulticopterGotoSetpointType(self)

    def on_activate(self):
        print("Mode activated")

    def on_deactivate(self):
        print("Mode deactivated")

    def update_setpoint(self, dt_s: float):
        self.setpoint.update((30, 30, -10), heading=px4_ros2.geometry.deg_to_rad(30))


def main(args=None):
    node = px4_ros2.Node("rlcpp_node", debug_output=True)
    mode = MyMode(node)
    assert mode.register()
    node.spin()


if __name__ == "__main__":
    main()
