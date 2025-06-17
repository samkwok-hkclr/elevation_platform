#include "elevation_platform/joint_motor_driver_node.hpp"

// namespace joint_motor_driver
// {

JointMotorDriverNode::JointMotorDriverNode(
  const rclcpp::NodeOptions& options)
: Node("joint_motor_driver_node", options)
{
  uint8_t can_id = 0;
  declare_parameter<uint8_t>("can_id", 0);
  declare_parameter<double>("gear_ratio", 0.0);

  get_parameter("can_id", can_id);
  get_parameter("gear_ratio", gear_ratio_);

  if (can_id == 0 || can_id > 0x7F)
  {
    RCLCPP_ERROR(get_logger(), "Incorrect CAN ID");
    return;
  }

  can_id_ = can_id;

  timer_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  can_sub_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions can_sub_options;
  can_sub_options.callback_group = can_sub_cbg_;

  base_status_timer_ = create_wall_timer(200ms, std::bind(&JointMotorDriverNode::base_status_cb, this), timer_cbg_);

  frames_pub_ = create_publisher<can_msgs::msg::Frame>("/to_can_bus", 1000);

  frames_sub_ = create_subscription<can_msgs::msg::Frame>(
    "/from_can_bus", 
    1000, 
    std::bind(&JointMotorDriverNode::can_frame_cb, this, _1),
    can_sub_options);

  std::stringstream ss;
  ss << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(can_id_);
  RCLCPP_INFO(get_logger(), "Joint Motor Driver Node [CAN-ID: %s] is up.", ss.str().c_str());
}
  
JointMotorDriverNode::~JointMotorDriverNode()
{

}

can_msgs::msg::Frame JointMotorDriverNode::create_status_frame(uint8_t command) 
{
  can_msgs::msg::Frame msg;
  msg.header.stamp = get_clock()->now();
  msg.id = can_id_;
  msg.dlc = 1;
  msg.data[0] = command;

  return msg;
}

can_msgs::msg::Frame JointMotorDriverNode::create_control_frame(uint8_t command, double value)
{
  can_msgs::msg::Frame frame;
  frame.id = can_id_;

  switch(command) 
  {
  case MotorCommands::SET_CURRENT_MODE:
  case MotorCommands::SET_MAX_POS_CURRENT:
  case MotorCommands::SET_MIN_NEG_CURRENT:
    // Current commands (mA)
    frame.dlc = 5;
    frame.data[0] = command;
    *reinterpret_cast<int32_t*>(&frame.data[1]) = static_cast<int32_t>(value);
    break;
  }

  return frame;
}

void JointMotorDriverNode::base_status_cb(void)
{
  for (const auto cmd : MotorCommands::CommandGroups::STATUS_COMMANDS) 
  {
    frames_pub_->publish(create_status_frame(cmd));
  }
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

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<JointMotorDriverNode>(options);

  exec->add_node(node->get_node_base_interface());
  exec->spin();

  rclcpp::shutdown();
}

// } // namespace joint_motor_driver

// #include "rclcpp_components/register_node_macro.hpp"

// RCLCPP_COMPONENTS_REGISTER_NODE(joint_motor_driver::JointMotorDriverNode)
