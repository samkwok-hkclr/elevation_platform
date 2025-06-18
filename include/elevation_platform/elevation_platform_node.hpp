#ifndef ELEVATION_PLATFORM_NODE_HPP_
#define ELEVATION_PLATFORM_NODE_HPP_

#pragma once

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "can_msgs/msg/frame.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class ElevationPlatformNode : public rclcpp::Node
{
public:
  explicit ElevationPlatformNode(const rclcpp::NodeOptions& options);
  ~ElevationPlatformNode() = default;
  
private:

};

#endif // ELEVATION_PLATFORM_NODE_HPP_