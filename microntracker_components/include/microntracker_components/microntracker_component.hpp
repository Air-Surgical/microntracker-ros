// Copyright 2025 Air Surgical, Inc.

#ifndef MICRONTRACKER_COMPONENTS__MICRONTRACKER_COMPONENT_HPP_
#define MICRONTRACKER_COMPONENTS__MICRONTRACKER_COMPONENT_HPP_

#include <string>

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
  void process_frames();

private:
  size_t count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  mtHandle CurrCamera;
  int CurrCameraSerialNum;
  mtHandle IdentifyingCamera;
};

int getMTHome(std::string & sMTHome);

}  // namespace microntracker_components

#endif  // MICRONTRACKER_COMPONENTS__MICRONTRACKER_COMPONENT_HPP_
