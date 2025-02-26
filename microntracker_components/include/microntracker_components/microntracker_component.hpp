// Copyright 2025 Air Surgical, Inc.

#ifndef MICRONTRACKER_COMPONENTS__MICRONTRACKER_COMPONENT_HPP_
#define MICRONTRACKER_COMPONENTS__MICRONTRACKER_COMPONENT_HPP_

#include <string>
#include <optional>

#include "microntracker_components/mtc_wrapper.hpp"
#include "microntracker_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

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
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  mtc::mtHandle CurrCamera;
  int CurrCameraSerialNum;
  mtc::mtHandle IdentifyingCamera;
  bool IsBackGroundProcessingEnabled;
};

std::optional<std::string> getMTHome();

}  // namespace microntracker_components

#endif  // MICRONTRACKER_COMPONENTS__MICRONTRACKER_COMPONENT_HPP_
