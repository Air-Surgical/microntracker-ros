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
  ~MicronTrackerDriver();

protected:
  void init_mtc();
  void on_timer();
  void process_frames();

private:
  bool IsBackGroundProcessingEnabled;
  int CurrCameraSerialNum;
  mtc::mtHandle CurrCamera;
  mtc::mtHandle IdentifiedMarkers;
  mtc::mtHandle IdentifyingCamera;
  mtc::mtHandle PoseXf;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

std::optional<std::string> getMTHome();

}  // namespace microntracker_components

#endif  // MICRONTRACKER_COMPONENTS__MICRONTRACKER_COMPONENT_HPP_
