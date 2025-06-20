#ifndef JOINT_MOTOR_DRIVER_NODE_HPP_
#define JOINT_MOTOR_DRIVER_NODE_HPP_

#pragma once

#include <functional>
#include <future>
#include <memory>
#include <chrono>
#include <iterator>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/empty.hpp"
#include "can_msgs/msg/frame.hpp"

#include "elevation_platform_msgs/msg/joint_motor_status.hpp"
#include "elevation_platform_msgs/srv/control_command.hpp"

#include "motor_command.hpp"
#include "command_group.hpp"
#include "motor_init_state.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class JointMotorDriverNode : public rclcpp::Node
{
  using Empty = std_msgs::msg::Empty;
  using Frame = can_msgs::msg::Frame;

  using JointMotorStatus = elevation_platform_msgs::msg::JointMotorStatus;
  using ControlCommand = elevation_platform_msgs::srv::ControlCommand;

public:
  explicit JointMotorDriverNode(const rclcpp::NodeOptions& options);
  ~JointMotorDriverNode();

  Frame create_one_byte_frame(const uint8_t command);
  Frame create_five_bytes_frame(const uint8_t command, const int32_t value);

  void init_cb(void);
  void base_status_cb(void);
  void pid_config_cb(void);
  void lim_config_cb(void);

  void pub_status_cb(void);

  int32_t parse_bytes(const uint8_t *val_ptr);
  void compose_bytes(uint8_t *val_ptr, const int32_t val);

  float velocity_convention(int32_t val) const;
  int32_t velocity_inverse_convention(float val) const;

  float position_convention(uint32_t val) const;

  void can_frame_cb(const Frame::SharedPtr msg);
  void halt_cb(const Empty::SharedPtr msg);

  void ctrl_cmd_handle(
    const std::shared_ptr<ControlCommand::Request> request, 
    std::shared_ptr<ControlCommand::Response> response);

private:
  std::mutex mutex_;
  
  uint8_t can_id_;
  float gear_ratio_;
  uint32_t position_offset_;
  int32_t max_current_;
  int32_t min_current_;

  MotorInitState state_;

  JointMotorStatus status_msg_;

  rclcpp::CallbackGroup::SharedPtr timer_cbg_;
  rclcpp::CallbackGroup::SharedPtr can_sub_cbg_;
  rclcpp::CallbackGroup::SharedPtr emergency_sub_cbg_;
  rclcpp::CallbackGroup::SharedPtr srv_ser_cbg_;

  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr base_status_timer_;
  rclcpp::TimerBase::SharedPtr pid_config_timer_;
  rclcpp::TimerBase::SharedPtr lim_config_timer_;
  rclcpp::TimerBase::SharedPtr pub_status_timer_;

  rclcpp::Publisher<Frame>::SharedPtr frames_pub_;
  rclcpp::Publisher<JointMotorStatus>::SharedPtr status_pub_;

  rclcpp::Subscription<Frame>::SharedPtr frames_sub_;
  rclcpp::Subscription<Empty>::SharedPtr halt_sub_;

  rclcpp::Service<ControlCommand>::SharedPtr ctrl_cmd_srv_;

  static constexpr float ENCODER_RESOLUTION = 262144.0; // 18-bit encoder
  static constexpr float VELOCITY_SCALE = 100.0;
  static constexpr float DEGREES_PER_REV = 360.0;
};


#endif // JOINT_MOTOR_DRIVER_NODE_HPP_