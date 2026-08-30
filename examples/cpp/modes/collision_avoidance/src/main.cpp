#include <rclcpp/rclcpp.hpp>
#include <mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>

using MyNodeWithMode = px4_ros2::NodeWithMode<WaypointFollowerMode>;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNodeWithMode>("waypoint_follower_node", true));
  rclcpp::shutdown();
  return 0;
}
