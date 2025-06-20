#include "elevation_platform/joint_motor_driver_node.hpp"

JointMotorDriverNode::JointMotorDriverNode(
  const rclcpp::NodeOptions& options)
: Node("joint_motor_driver_node", options)
{
  declare_parameter<uint8_t>("can_id", 0);
  declare_parameter<float>("gear_ratio", 0.0);
  declare_parameter<int32_t>("position_offset", 0);
  declare_parameter<int32_t>("max_current", 0);
  declare_parameter<int32_t>("min_current", 0);

  get_parameter("can_id", can_id_);
  get_parameter("gear_ratio", gear_ratio_);
  get_parameter("position_offset", position_offset_);
  get_parameter("max_current", max_current_);
  get_parameter("min_current", min_current_);

  if (can_id_ == 0 || can_id_ > 0x7F)
  {
    RCLCPP_ERROR(get_logger(), "CAN ID does not set");
    rclcpp::shutdown();
    return;
  }

  if (gear_ratio_ == 0.0)
  {
    RCLCPP_ERROR(get_logger(), "gear ratio does not set");
    rclcpp::shutdown();
    return;
  }

  status_msg_.can_id = can_id_;
  status_msg_.gear_ratio = gear_ratio_;

  timer_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  can_sub_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  emergency_sub_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_ser_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions can_sub_options;
  can_sub_options.callback_group = can_sub_cbg_;
  rclcpp::SubscriptionOptions halt_sub_options;
  halt_sub_options.callback_group = emergency_sub_cbg_;

  // init_timer_ = create_wall_timer(200ms, std::bind(&JointMotorDriverNode::init_cb, this), timer_cbg_);
  base_status_timer_ = create_wall_timer(200ms, std::bind(&JointMotorDriverNode::base_status_cb, this), timer_cbg_);
  pid_config_timer_ = create_wall_timer(1000ms, std::bind(&JointMotorDriverNode::pid_config_cb, this), timer_cbg_);
  lim_config_timer_ = create_wall_timer(1000ms, std::bind(&JointMotorDriverNode::lim_config_cb, this), timer_cbg_);
  pub_status_timer_ = create_wall_timer(200ms, std::bind(&JointMotorDriverNode::pub_status_cb, this), timer_cbg_);

  frames_pub_ = create_publisher<Frame>("/to_can_bus", 1000);
  status_pub_ = create_publisher<JointMotorStatus>("/elevate_joint_motor_status", 10);

  frames_sub_ = create_subscription<Frame>(
    "/from_can_bus", 
    1000, 
    std::bind(&JointMotorDriverNode::can_frame_cb, this, _1),
    can_sub_options);

  halt_sub_ = create_subscription<Empty>(
    "halt", 
    10, 
    std::bind(&JointMotorDriverNode::halt_cb, this, _1),
    halt_sub_options);

  ctrl_cmd_srv_ = create_service<ControlCommand>(
    "control_command", 
    std::bind(&JointMotorDriverNode::ctrl_cmd_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  RCLCPP_INFO(get_logger(), "Joint Motor Driver Node [CAN-ID: 0x%02X] is up.", static_cast<int>(can_id_));
}
  
JointMotorDriverNode::~JointMotorDriverNode()
{

}

can_msgs::msg::Frame JointMotorDriverNode::create_one_byte_frame(const uint8_t command) 
{
  Frame msg(rosidl_runtime_cpp::MessageInitialization::ZERO);

  msg.header.stamp = get_clock()->now();
  msg.id = can_id_;
  msg.dlc = 1;
  msg.data[0] = command;

  return msg;
}

can_msgs::msg::Frame JointMotorDriverNode::create_five_bytes_frame(const uint8_t command, const int32_t value)
{
  Frame msg(rosidl_runtime_cpp::MessageInitialization::ZERO);

  msg.header.stamp = get_clock()->now();
  msg.id = can_id_;
  msg.dlc = 5;
  msg.data[0] = command;

  switch(command) 
  {
  case MotorCommand::SET_CURRENT_MODE:
    // TBD
    break;
  case MotorCommand::SET_VELOCITY_MODE:
    // TBD
    break;
  case MotorCommand::SET_POSITION_MODE:
    compose_bytes(&msg.data[1], value);
    break;
  // case XXX:
  //   break;
  default:
    RCLCPP_WARN(get_logger(), "Unhandled CMD: 0x%02X", static_cast<int>(command));
    break;
  }

  return msg;
}

void JointMotorDriverNode::init_cb(void)
{
  if (!state_.is_pos_offset_initialized())
  {
    frames_pub_->publish(create_five_bytes_frame(MotorCommand::SET_POSITION_OFFSET, position_offset_));
  }
  if (state_.all_initialized())
  {
    init_timer_->cancel();
    RCLCPP_WARN(get_logger(), "Joint Motor [CAN-ID: 0x%02X] Initialized.", static_cast<int>(can_id_));
  }
}

void JointMotorDriverNode::base_status_cb(void)
{
  for (const auto &cmd : CommandGroups::GET_STATUS_CMDS) 
  {
    frames_pub_->publish(create_one_byte_frame(cmd));
  }
}

void JointMotorDriverNode::pid_config_cb(void)
{
  for (const auto &cmd : CommandGroups::GET_PID_CMDS) 
  {
    frames_pub_->publish(create_one_byte_frame(cmd));
  }
}

void JointMotorDriverNode::lim_config_cb(void)
{
  for (const auto &cmd : CommandGroups::GET_LIMIT_CMDS) 
  {
    frames_pub_->publish(create_one_byte_frame(cmd));
  }
}

void JointMotorDriverNode::pub_status_cb(void)
{
  const std::lock_guard<std::mutex> lock(mutex_);

  status_msg_.header.stamp = get_clock()->now();
  status_pub_->publish(status_msg_);
}

void JointMotorDriverNode::ctrl_cmd_handle(
  const std::shared_ptr<ControlCommand::Request> request, 
  std::shared_ptr<ControlCommand::Response> response)
{
  frames_pub_->publish(create_five_bytes_frame(request->command, request->value));

  response->success = true;
}

void JointMotorDriverNode::can_frame_cb(const Frame::SharedPtr msg)
{
  if (static_cast<uint8_t>(msg->id) != can_id_)
    return;

  const std::lock_guard<std::mutex> lock(mutex_);
  
  switch (msg->data[0])
  {
  case MotorCommand::GET_MODE:
    /* code */
    break;
  case MotorCommand::GET_CURRENT:
    status_msg_.current = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_TARGET_CURRENT:
    status_msg_.target_current = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_VELOCITY:
    status_msg_.velocity = velocity_convention(parse_bytes(&msg->data[1]));
    break;
  case MotorCommand::GET_TARGET_VELOCITY:
    status_msg_.target_velocity = velocity_convention(parse_bytes(&msg->data[1]));
    break;
  case MotorCommand::GET_POSITION:
  {
    int32_t tmp = parse_bytes(&msg->data[1]);
    status_msg_.current_degree = position_convention(tmp);
    status_msg_.position = tmp;
  }
    break;
  case MotorCommand::GET_TARGET_POSITION:
    status_msg_.target_position = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_ERROR_STATUS:
    /* code */
    break;
  case MotorCommand::GET_BUS_VOLTAGE:
    status_msg_.bus_voltage = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_MOTOR_TEMP:
    status_msg_.motor_temp = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_BOARD_TEMP:
    status_msg_.board_temp = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_ENCODER_VOLTAGE:
    status_msg_.encoder_voltage = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_ENCODER_STATUS:
    status_msg_.encoder_status = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::SET_POSITION_OFFSET:
    state_.set_pos_offset_initialized(true);
    break;
  // case XXXX:
  //   break;
  default:
    RCLCPP_WARN(get_logger(), "Unhandled CMD: 0x%02X", static_cast<int>(msg->data[0]));
    break;
  }
}

void JointMotorDriverNode::halt_cb(const Empty::SharedPtr msg)
{
  (void) msg;
  frames_pub_->publish(create_one_byte_frame(MotorCommand::MOTOR_STOP));

  RCLCPP_WARN(get_logger(), "Joint Motor [CAN-ID: 0x%02X] halted", static_cast<int>(can_id_));
}

// ==================== byte convention ====================

int32_t JointMotorDriverNode::parse_bytes(const uint8_t *val_ptr)
{
  if (!val_ptr) 
  {
    RCLCPP_ERROR(get_logger(), "Null pointer in parse bytes");
    return 0;
  }

  return val_ptr[0] | (val_ptr[1] << 8) | (val_ptr[2] << 16) | (val_ptr[3] << 24);
}

void JointMotorDriverNode::compose_bytes(uint8_t *val_ptr, const int32_t val)
{
  if (!val_ptr) 
  {
    RCLCPP_ERROR(get_logger(), "Null pointer in compose bytes");
    return;
  }

  val_ptr[0] = static_cast<uint8_t>(val       & 0xFF);
  val_ptr[1] = static_cast<uint8_t>(val >> 8  & 0xFF);
  val_ptr[2] = static_cast<uint8_t>(val >> 16 & 0xFF);
  val_ptr[3] = static_cast<uint8_t>(val >> 24 & 0xFF);
}

// ==================== velocity convention ====================

float JointMotorDriverNode::velocity_convention(int32_t val) const
{
  return static_cast<float>(val) / VELOCITY_SCALE / gear_ratio_ * DEGREES_PER_REV;
}

int32_t JointMotorDriverNode::velocity_inverse_convention(float val) const
{
  return static_cast<int32_t>((val * gear_ratio_ * VELOCITY_SCALE ) / DEGREES_PER_REV);
}

// ==================== position convention ====================

float JointMotorDriverNode::position_convention(uint32_t val) const
{
  return static_cast<float>(val) / ENCODER_RESOLUTION * DEGREES_PER_REV;
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