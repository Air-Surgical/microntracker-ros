// Copyright 2025 Air Surgical, Inc.

#ifndef MICRONTRACKER_COMPONENTS__MICRONTRACKER_COMPONENT_HPP_
#define MICRONTRACKER_COMPONENTS__MICRONTRACKER_COMPONENT_HPP_

#include <thread>
#include <memory>
#include <string>

// #include "opencv2/core/core.hpp"
#include "opencv2/calib3d.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "microntracker_components/microntracker_components_parameters.hpp"
#include "microntracker_components/mtc_wrapper.hpp"
#include "microntracker_components/visibility_control.h"

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
  void process_frame();
  void publish_images(const std_msgs::msg::Header & header);
  void publish_markers(const std_msgs::msg::Header & header);

private:
  bool is_alive;
  int CurrCameraSerialNum;
  mtc::mtHandle CurrCamera;
  mtc::mtHandle IdentifiedMarkers;
  mtc::mtHandle IdentifyingCamera;
  mtc::mtHandle PoseXf;
  Params params_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_right_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_right_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
  rclcpp::Time mt_epoch;
  size_t count_;
  std::shared_ptr<ParamListener> param_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::thread process_thread;
};

}  // namespace microntracker_components

#endif  // MICRONTRACKER_COMPONENTS__MICRONTRACKER_COMPONENT_HPP_
