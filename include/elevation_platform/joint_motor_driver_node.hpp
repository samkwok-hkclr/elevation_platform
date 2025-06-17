#ifndef JOINT_MOTOR_DRIVER_NODE_HPP_
#define JOINT_MOTOR_DRIVER_NODE_HPP_

#pragma once

#include <functional>
#include <future>
#include <memory>
#include <chrono>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
// #include "rclcpp_components/register_node_macro.hpp"

#include "can_msgs/msg/frame.hpp"

#include "motor_commands.hpp"

// namespace joint_motor_driver
// {

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class JointMotorDriverNode : public rclcpp::Node
{
public:
  explicit JointMotorDriverNode(const rclcpp::NodeOptions& options);
  ~JointMotorDriverNode();

  can_msgs::msg::Frame create_status_frame(uint8_t command);
  can_msgs::msg::Frame create_control_frame(uint8_t command, double value);

  void base_status_cb(void);
  void can_frame_cb(const can_msgs::msg::Frame::SharedPtr msg);

private:
  std::mutex mutex_;
  uint8_t can_id_;
  double gear_ratio_;

  rclcpp::CallbackGroup::SharedPtr timer_cbg_;
  rclcpp::CallbackGroup::SharedPtr can_sub_cbg_;

  rclcpp::TimerBase::SharedPtr base_status_timer_;

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr frames_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr frames_sub_;

  static constexpr double ENCODER_RESOLUTION = 262144.0; // 18-bit encoder
  static constexpr double VELOCITY_SCALE = 100.0;
};

// } // namespace joint_motor_driver

#endif // JOINT_MOTOR_DRIVER_NODE_HPP_