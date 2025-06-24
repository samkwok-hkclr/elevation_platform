#include "elevation_platform/elevation_platform_node.hpp"

ElevationPlatformNode::ElevationPlatformNode(
	const rclcpp::NodeOptions& options)
: Node("elevation_platform_node", options)
{
  RCLCPP_INFO(get_logger(), "Elevation Platform Node is up.");
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<ElevationPlatformNode>(options);

  exec->add_node(node->get_node_base_interface());
  exec->spin();

  rclcpp::shutdown();
}