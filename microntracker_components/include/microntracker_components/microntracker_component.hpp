// Copyright 2025 Air Surgical, Inc.

#ifndef MICRONTRACKER_COMPONENTS__MICRONTRACKER_COMPONENT_HPP_
#define MICRONTRACKER_COMPONENTS__MICRONTRACKER_COMPONENT_HPP_

#include "microntracker_components/visibility_control.h"
#include "microntracker/MTC.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace microntracker_components
{

class MicronTrackerDriver : public rclcpp::Node
{
public:
  MICRONTRACKER_COMPONENTS_PUBLIC
  explicit MicronTrackerDriver(const rclcpp::NodeOptions & options);

protected:
  void on_timer();

private:
  size_t count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace microntracker_components

#endif  // MICRONTRACKER_COMPONENTS__MICRONTRACKER_COMPONENT_HPP_
