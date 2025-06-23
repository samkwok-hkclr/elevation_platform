#include "elevation_platform/joint_motor_driver_node.hpp"

JointMotorDriverNode::JointMotorDriverNode(
  const rclcpp::NodeOptions& options)
: Node("joint_motor_driver_node", options)
{
  declare_parameter<uint8_t>("can_id", 0);
  declare_parameter<float>("gear_ratio", 0.0);
  declare_parameter<int32_t>("position_offset", 0);

  declare_parameter<int32_t>("position_p", 0);
  declare_parameter<int32_t>("position_d", 0);
  declare_parameter<int32_t>("velocity_p", 0);
  declare_parameter<int32_t>("velocity_i", 0);

  declare_parameter<int32_t>("max_forward_current", 0);
  declare_parameter<int32_t>("min_backward_current", 0);
  declare_parameter<double>("max_forward_velocity", 0);
  declare_parameter<double>("max_backward_velocity", 0);

  get_parameter("can_id", can_id_);
  get_parameter("gear_ratio", gear_ratio_);

  status_.position_offset = get_parameter("position_offset").as_int();

  status_.position_p = get_parameter("position_p").as_int();
  status_.position_d = get_parameter("position_d").as_int();
  status_.velocity_p = get_parameter("velocity_p").as_int();
  status_.velocity_i = get_parameter("velocity_i").as_int();

  status_.max_forward_current = get_parameter("max_forward_current").as_int();
  status_.min_backward_current = get_parameter("min_backward_current").as_int();
  status_.max_forward_velocity = get_parameter("max_forward_velocity").as_double();
  status_.min_backward_velocity = get_parameter("max_backward_velocity").as_double();

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

  timer_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  can_sub_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  emergency_sub_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_ser_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions can_sub_options;
  can_sub_options.callback_group = can_sub_cbg_;
  rclcpp::SubscriptionOptions halt_sub_options;
  halt_sub_options.callback_group = emergency_sub_cbg_;

  init_timer_ = create_wall_timer(500ms, std::bind(&JointMotorDriverNode::init_cb, this), timer_cbg_);
  base_status_timer_ = create_wall_timer(200ms, std::bind(&JointMotorDriverNode::base_status_cb, this), timer_cbg_);
  pid_config_timer_ = create_wall_timer(1000ms, std::bind(&JointMotorDriverNode::pid_config_cb, this), timer_cbg_);
  lim_config_timer_ = create_wall_timer(1000ms, std::bind(&JointMotorDriverNode::lim_config_cb, this), timer_cbg_);
  pub_status_timer_ = create_wall_timer(500ms, std::bind(&JointMotorDriverNode::pub_status_cb, this), timer_cbg_);

  frames_pub_ = create_publisher<Frame>("/to_can_bus", 1000);
  status_pub_ = create_publisher<JointMotorStatus>("/elevation_joint_motor_status", 10);

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

  clear_sub_ = create_subscription<Empty>(
    "clear", 
    10, 
    std::bind(&JointMotorDriverNode::clear_cb, this, _1));

  deg_rotate_sub_ = create_subscription<Float32>(
    "rotate_in_degree", 
    10, 
    std::bind(&JointMotorDriverNode::deg_rotate_cb, this, _1));

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

can_msgs::msg::Frame JointMotorDriverNode::create_five_bytes_frame(const uint8_t command, const uint32_t value)
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
  case MotorCommand::SET_MAX_VELOCITY:
  case MotorCommand::SET_MIN_VELOCITY:
    compose_bytes(&msg.data[1], value);
    break;
  case MotorCommand::SET_MAX_POS_CURRENT:
  case MotorCommand::SET_MIN_NEG_CURRENT:
    compose_bytes(&msg.data[1], value);
    break;
  case MotorCommand::SET_POSITION_P:
  case MotorCommand::SET_POSITION_D:
  case MotorCommand::SET_VELOCITY_P:
  case MotorCommand::SET_VELOCITY_I:
    compose_bytes(&msg.data[1], value);
    break;
  default:
    RCLCPP_WARN(get_logger(), "Unhandled in five bytes CMD: 0x%02X", static_cast<int>(command));
    break;
  }
  RCLCPP_DEBUG(get_logger(), "composed a value to bytes: %d", value);

  return msg;
}

void JointMotorDriverNode::init_cb(void)
{
  frames_pub_->publish(
    create_five_bytes_frame(
      MotorCommand::SET_POSITION_P, 
      static_cast<uint32_t>(status_.position_p)));
  frames_pub_->publish(
    create_five_bytes_frame(
      MotorCommand::SET_POSITION_D, 
      static_cast<uint32_t>(status_.position_d)));

  frames_pub_->publish(
    create_five_bytes_frame(
      MotorCommand::SET_VELOCITY_P, 
      static_cast<uint32_t>(status_.velocity_p)));
  frames_pub_->publish(
    create_five_bytes_frame(
      MotorCommand::SET_VELOCITY_I, 
      static_cast<uint32_t>(status_.velocity_i)));

  frames_pub_->publish(
    create_five_bytes_frame(
      MotorCommand::SET_MAX_POS_CURRENT, 
      static_cast<uint32_t>(status_.max_forward_current)));
  frames_pub_->publish(
    create_five_bytes_frame(
      MotorCommand::SET_MIN_NEG_CURRENT, 
      static_cast<uint32_t>(status_.min_backward_current)));

  frames_pub_->publish(
    create_five_bytes_frame(
      MotorCommand::SET_MAX_VELOCITY, 
      velocity_inverse_convention(status_.max_forward_velocity)));
  frames_pub_->publish(
    create_five_bytes_frame(
      MotorCommand::SET_MIN_VELOCITY, 
      velocity_inverse_convention(status_.min_backward_velocity)));
  
  RCLCPP_INFO(get_logger(), "Joint Motor Driver Node [CAN-ID: 0x%02X] is initialized.", static_cast<int>(can_id_));

  init_timer_->cancel();
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
  JointMotorStatus msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
  msg.header.stamp = get_clock()->now();

  const std::lock_guard<std::mutex> lock(mutex_);

  msg.can_id                = can_id_;
  msg.gear_ratio            = gear_ratio_;
  msg.current_degree        = status_.current_degree;
  msg.mode                  = status_.operation_mode;
  msg.current               = status_.current;
  msg.target_current        = status_.target_current;
  msg.velocity              = status_.velocity;
  msg.target_velocity       = status_.target_velocity;
  msg.position              = status_.position;
  msg.target_position       = status_.target_position;
  msg.error_status          = status_.error_status;
  msg.bus_voltage           = status_.bus_voltage;

  msg.motor_temp            = status_.motor_temp;
  msg.board_temp            = status_.board_temp;

  msg.encoder_voltage       = status_.encoder_voltage;
  msg.encoder_status        = status_.encoder_status;

  msg.max_forward_velocity  = status_.max_forward_velocity;
  msg.min_backward_velocity = status_.min_backward_velocity;
  msg.max_forward_position  = status_.max_forward_position;
  msg.min_backward_position = status_.min_backward_position;

  msg.position_p = status_.position_p;
  msg.position_i = status_.position_i;
  msg.position_d = status_.position_d;
  msg.velocity_p = status_.velocity_p;
  msg.velocity_i = status_.velocity_i;
  msg.velocity_d = status_.velocity_d;
  msg.current_p = status_.current_p;
  msg.current_i = status_.current_i;
  msg.current_d = status_.current_d;

  status_pub_->publish(msg);
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
  case MotorCommand::CLEAR_ERRORS:
  case MotorCommand::MOTOR_STOP:
    break;
  case MotorCommand::GET_MODE:
    status_.operation_mode = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_CURRENT:
    status_.current = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_TARGET_CURRENT:
    status_.target_current = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_VELOCITY:
    status_.velocity = velocity_convention(parse_bytes(&msg->data[1]));
    break;
  case MotorCommand::GET_TARGET_VELOCITY:
    status_.target_velocity = velocity_convention(parse_bytes(&msg->data[1]));
    break;
  case MotorCommand::GET_POSITION:
  {
    uint32_t tmp = static_cast<uint32_t>(parse_bytes(&msg->data[1]));
    status_.current_degree = position_convention(tmp);
    status_.position = tmp;
  }
    break;
  case MotorCommand::GET_TARGET_POSITION:
    status_.target_position = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_ERROR_STATUS:
    status_.error_status = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_BUS_VOLTAGE:
    status_.bus_voltage = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_MOTOR_TEMP:
    status_.motor_temp = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_BOARD_TEMP:
    status_.board_temp = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_ENCODER_VOLTAGE:
    status_.encoder_voltage = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_ENCODER_STATUS:
    status_.encoder_status = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::SET_POSITION_OFFSET:
    status_.position_offset = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_VELOCITY_P:
    status_.velocity_p = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_VELOCITY_I:
    status_.velocity_i = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_VELOCITY_D:
    status_.velocity_d = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_POSITION_P:
    status_.position_p = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_POSITION_I:
    status_.position_i = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_POSITION_D:
    status_.position_d = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_MAX_CURRENT:
    status_.max_current_abs = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_MAX_POS_CURRENT:
    status_.max_forward_current = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_MAX_NEG_CURRENT:
    status_.min_backward_current = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_MAX_ACCEL:
    status_.max_forward_accel = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_MIN_ACCEL:
    status_.min_backward_accel = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_MAX_VELOCITY:
    status_.max_forward_velocity = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_MIN_VELOCITY:
    status_.min_backward_velocity = parse_bytes(&msg->data[1]);
    break;
  case MotorCommand::GET_MAX_POSITION:
    status_.max_forward_position = static_cast<uint32_t>(parse_bytes(&msg->data[1]));
    break;
  case MotorCommand::GET_MIN_POSITION:
    status_.min_backward_position = static_cast<uint32_t>(parse_bytes(&msg->data[1]));
    break;
  case MotorCommand::GET_OVERVOLT_THRESH:
  case MotorCommand::GET_UNDERVOLT_THRESH:
    break;
  case MotorCommand::GET_MOTOR_OT_THRESH:
  case MotorCommand::GET_BOARD_OT_THRESH:
    break;
  case MotorCommand::SET_MAX_VELOCITY:
  case MotorCommand::SET_MIN_VELOCITY:
    break;
  case MotorCommand::SET_POSITION_MODE:
    break;
  case MotorCommand::SET_MAX_POS_CURRENT:
  case MotorCommand::SET_MIN_NEG_CURRENT:
    break;
  case MotorCommand::SET_POSITION_P:
  case MotorCommand::SET_POSITION_D:
  case MotorCommand::SET_VELOCITY_P:
  case MotorCommand::SET_VELOCITY_I:
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
  frames_pub_->publish(create_one_byte_frame(MotorCommand::MOTOR_STOP));
  (void) msg;

  RCLCPP_WARN(get_logger(), "Joint Motor [CAN-ID: 0x%02X] halted", static_cast<int>(can_id_));
}

void JointMotorDriverNode::clear_cb(const Empty::SharedPtr msg)
{
  frames_pub_->publish(create_one_byte_frame(MotorCommand::CLEAR_ERRORS));
  (void) msg;

  RCLCPP_WARN(get_logger(), "Joint Motor [CAN-ID: 0x%02X] clear error", static_cast<int>(can_id_));
}

void JointMotorDriverNode::deg_rotate_cb(const Float32::SharedPtr msg)
{
  // for debug purpose only
  const float MAX_DEBUG_SAFETY = 90.0;
  const float MIN_DEBUG_SAFETY = -90.0;

  if (msg->data > MAX_DEBUG_SAFETY || msg->data < MIN_DEBUG_SAFETY)
    return;

  uint32_t rotation_steps = static_cast<uint32_t>(ENCODER_RESOLUTION * abs(msg->data) / DEGREES_PER_REV);
  uint32_t target_position = 0;

  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (msg->data > 0.0f)
      target_position = status_.position + rotation_steps;
    else
      target_position = status_.position - rotation_steps;
  }

  frames_pub_->publish(
    create_five_bytes_frame(
      MotorCommand::SET_POSITION_MODE,
      target_position));

  RCLCPP_DEBUG(get_logger(), "current position: %d", status_.position);
  RCLCPP_DEBUG(get_logger(), "rotation_steps: %d", rotation_steps);
  RCLCPP_DEBUG(get_logger(), "target position: %d", target_position);

  RCLCPP_WARN(get_logger(), "Joint Motor [CAN-ID: 0x%02X] rotated %.2f deg", static_cast<int>(can_id_), msg->data);
}

// ==================== byte convention ====================

int32_t JointMotorDriverNode::parse_bytes(const uint8_t *val_ptr)
{
  if (!val_ptr) 
  {
    RCLCPP_ERROR(get_logger(), "Null pointer in parse bytes");
    return 0;
  }

  int32_t tmp = val_ptr[0] | (val_ptr[1] << 8) | (val_ptr[2] << 16) | (val_ptr[3] << 24);

  RCLCPP_DEBUG(get_logger(), "Valve: %d", tmp);
  return tmp;
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
  float tmp = static_cast<int32_t>(val) / ENCODER_RESOLUTION * DEGREES_PER_REV;
  RCLCPP_DEBUG(get_logger(), "position_convention: %.4f", tmp);

  return tmp;
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