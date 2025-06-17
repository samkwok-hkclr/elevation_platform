#include "elevation_platform/joint_motor_driver_node.hpp"

namespace joint_motor_driver
{

JointMotorDriverNode::JointMotorDriverNode(const rclcpp::NodeOptions& options, uint8_t can_id)
: Node("joint_motor_driver_node", options),
{
  if (can_id > 0x7F)
  {
    RCLCPP_ERROR(get_logger(), "Incorrect CAN ID");
    return;
  }

  can_id_ = can_id;

  frames_pub_ = this->create_publisher<can_msgs::msg::Frame>("to_can_bus", 1000);
  frames_sub_ = this->create_subscription<can_msgs::msg::Frame>(
    "from_can_bus", 
    1000, 
    std::bind(&JointMotorDriverNode::can_frame_cb, this, _1));
}

JointMotorDriverNode::~JointMotorDriverNode()
{

}

void JointMotorDriverNode::can_frame_cb(const can_msgs::msg::Frame::SharedPtr msg)
{
  if (msg->id != can_id_)
    return;
  
  switch (msg->data[0])
  {
  case 0x11:
    /* code */
    break;
  
  default:
    break;
  }
}

} // namespace joint_motor_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(joint_motor_driver::JointMotorDriverNode)
