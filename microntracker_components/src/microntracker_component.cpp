// Copyright 2025 Air Surgical, Inc.

#include "microntracker_components/microntracker_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <filesystem>
#include <string>
#include <optional>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace microntracker_components
{

MicronTrackerDriver::MicronTrackerDriver(const rclcpp::NodeOptions & options)
: Node("microntracker_driver", options), count_(0)
{
  marker_array_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
  left_image_pub_ = create_publisher<sensor_msgs::msg::Image>("left_image", 10);
  right_image_pub_ = create_publisher<sensor_msgs::msg::Image>("right_image", 10);

  param_listener_ = std::make_shared<ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // Initialize MTC library and connect to cameras
  init_mtc();

  // while node is running, process frames
  while (rclcpp::ok()) {
    process_frames();
  }
}

MicronTrackerDriver::~MicronTrackerDriver()
{
  mtc::Cameras_Detach();
  mtc::Camera_Free(CurrCamera);
  mtc::Collection_Free(IdentifiedMarkers);
  mtc::Xform3D_Free(PoseXf);
}

void MicronTrackerDriver::init_mtc()
{
  // Initialize MTC library and connect to cameras
  auto calibrationDir = mtr::getMTHome();
  if (!calibrationDir) {
    RCLCPP_ERROR(this->get_logger(), "MTHome environment variable not set");
    return;
  }

  auto markerDir = fs::path(*calibrationDir) / "Markers";
  auto calibrationDirPath = fs::path(*calibrationDir) / "CalibrationFiles";

  MTR(mtc::Cameras_AttachAvailableCameras(calibrationDirPath.c_str()));
  if (mtc::Cameras_Count() < 1) {
    RCLCPP_ERROR(this->get_logger(), "No camera found!");
    return;
  }

  MTR(mtc::Cameras_ItemGet(0, &CurrCamera));
  MTR(mtc::Camera_SerialNumberGet(CurrCamera, &CurrCameraSerialNum));
  RCLCPP_INFO(this->get_logger(), "Attached %d camera(s). Curr camera is %d", mtc::Cameras_Count(),
              CurrCameraSerialNum);

  mtc::mtStreamingModeStruct mode{
    mtr::stringToFrameType(params_.mt.frame_type),
    mtr::stringToDecimation(params_.mt.decimation),
    mtr::stringToBitDepth(params_.mt.bit_depth)};

  MTR(mtc::Cameras_StreamingModeSet(mode, CurrCameraSerialNum));

  auto t0 = now();
  mtc::ResetMTTime();
  auto t1 = now();
  mt_epoch = t0 + (t1 - t0) * 0.5;

  int width, height;
  MTR(mtc::Camera_ResolutionGet(CurrCamera, &width, &height));
  RCLCPP_INFO(this->get_logger(), "The camera resolution is %d x %d", width, height);

  IsBackGroundProcessingEnabled = false;
  if (IsBackGroundProcessingEnabled) {
    MTR(mtc::Markers_BackGroundProcessSet(true));
    RCLCPP_INFO(this->get_logger(), "Background processing enabled");
  }

  MTR(mtc::Markers_LoadTemplates(const_cast<char *>(markerDir.c_str())));
  RCLCPP_INFO(this->get_logger(), "Loaded %d marker templates", mtc::Markers_TemplatesCount());

  IdentifiedMarkers = mtc::Collection_New();
  PoseXf = mtc::Xform3D_New();
}

void MicronTrackerDriver::process_frames()
{
  auto msg = std::make_unique<visualization_msgs::msg::MarkerArray>();

  if (IsBackGroundProcessingEnabled) {
    mtc::Markers_GetIdentifiedMarkersFromBackgroundThread(CurrCamera);
  } else {
    MTR(mtc::Cameras_GrabFrame(0));
    MTR(mtc::Markers_ProcessFrame(0));
  }

  double frame_secs;
  mtc::Camera_FrameMTTimeSecsGet(CurrCamera, &frame_secs);
  auto stamp = [this, frame_secs]() {
      auto duration = rclcpp::Duration::from_seconds(frame_secs);
      return this->mt_epoch + duration;
    }();
  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = params_.frame_id;

  int width, height, step;
  MTR(mtc::Camera_ResolutionGet(CurrCamera, &width, &height));
  width /= 4;
  height /= 4;
  step = width * 3;
  bool is_bigendian = false;
  int image_buffer_size = (width * height) * 3;

  std::vector<unsigned char> left_image_data(image_buffer_size);
  std::vector<unsigned char> right_image_data(image_buffer_size);
  std::string encoding = "rgb8";
  mtc::Camera_24BitQuarterSizeImagesGet(
    CurrCamera, left_image_data.data(), right_image_data.data());

  auto publish_image = [&](
    const std::vector<unsigned char> & image_data,
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & publisher) {
      auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
      image_msg->header = header;
      image_msg->height = height;
      image_msg->width = width;
      image_msg->encoding = encoding;
      image_msg->is_bigendian = is_bigendian;
      image_msg->step = step;
      image_msg->data = image_data;
      publisher->publish(std::move(image_msg));
    };

  // Publish left image
  publish_image(left_image_data, left_image_pub_);

  // Publish right image
  publish_image(right_image_data, right_image_pub_);

  MTR(mtc::Markers_IdentifiedMarkersGet(0, IdentifiedMarkers));
  auto clock = this->get_clock();
  RCLCPP_INFO_THROTTLE(this->get_logger(), *clock, 1000, "identified %d marker(s)",
                       mtc::Collection_Count(IdentifiedMarkers));
  std::vector<double> position(3);
  std::vector<double> orientation(4);

  for (int j = 1; j <= mtc::Collection_Count(IdentifiedMarkers); j++) {
    mtc::mtHandle Marker = mtc::Collection_Int(IdentifiedMarkers, j);
    MTR(mtc::Marker_Marker2CameraXfGet(Marker, CurrCamera, PoseXf, &IdentifyingCamera));

    if (IdentifyingCamera != 0) {
      char MarkerName[MT_MAX_STRING_LENGTH];
      // mtc::mtMeasurementHazardCode Hazard;

      MTR(mtc::Marker_NameGet(Marker, MarkerName, MT_MAX_STRING_LENGTH, 0));
      MTR(mtc::Xform3D_ShiftGet(PoseXf, position.data()));
      MTR(mtc::Xform3D_RotQuaternionsGet(PoseXf, orientation.data()));
      // MTR(mtc::Xform3D_HazardCodeGet(PoseXf, &Hazard));

      visualization_msgs::msg::Marker marker;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.id = j;
      marker.header = header;
      marker.text = MarkerName;
      marker.pose.position.x = position[0] / 1000;
      marker.pose.position.y = position[1] / 1000;
      marker.pose.position.z = position[2] / 1000;
      marker.pose.orientation.x = orientation[0];
      marker.pose.orientation.y = orientation[1];
      marker.pose.orientation.z = orientation[2];
      marker.pose.orientation.w = orientation[3];
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      msg->markers.push_back(marker);
    }
  }

  // Put the message into a queue to be processed by the middleware.
  // This call is non-blocking.
  marker_array_pub_->publish(std::move(msg));
}

}  // namespace microntracker_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(microntracker_components::MicronTrackerDriver)
