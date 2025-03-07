// Copyright 2025 Air Surgical, Inc.

#ifndef MICRONTRACKER_COMPONENTS__MICRONTRACKER_COMPONENT_HPP_
#define MICRONTRACKER_COMPONENTS__MICRONTRACKER_COMPONENT_HPP_

#include <memory>
#include <optional>
#include <string>

#include "microntracker_components/microntracker_components_parameters.hpp"
#include "microntracker_components/mtc_wrapper.hpp"
#include "microntracker_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
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
  void process_frames();

private:
  bool IsBackGroundProcessingEnabled;
  int CurrCameraSerialNum;
  mtc::mtHandle CurrCamera;
  mtc::mtHandle IdentifiedMarkers;
  mtc::mtHandle IdentifyingCamera;
  mtc::mtHandle PoseXf;
  Params params_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
  rclcpp::Time mt_epoch;
  size_t count_;
  std::shared_ptr<ParamListener> param_listener_;
};

}  // namespace microntracker_components

#endif  // MICRONTRACKER_COMPONENTS__MICRONTRACKER_COMPONENT_HPP_
