#ifndef JOINT_MOTOR_DRIVER_NODE_HPP_
#define JOINT_MOTOR_DRIVER_NODE_HPP_

#include <functional>
#include <future>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "can_msgs/msg/frame.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace joint_motor_driver
{

class JointMotorDriverNode : public rclcpp::Node
{
public:
  explicit JointMotorDriverNode(const rclcpp::NodeOptions& options, uint8_t can_id);
  ~JointMotorDriverNode();

  void can_frame_cb(const can_msgs::msg::Frame::SharedPtr msg);

private:
  uint8_t can_id_;

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr frames_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr frames_sub_;

};

} // namespace joint_motor_driver

#endif // JOINT_MOTOR_DRIVER_NODE_HPP_