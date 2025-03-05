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

// Macro to check for and report MTC usage errors.
#define MTC(func) {int r = func; \
  if (r != mtc::mtOK) RCLCPP_ERROR(this->get_logger(), "MTC error: %s", mtc::MTLastErrorString());}

MicronTrackerDriver::MicronTrackerDriver(const rclcpp::NodeOptions & options)
: Node("microntracker_driver", options), count_(0)
{
  marker_array_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
  left_image_pub_ = create_publisher<sensor_msgs::msg::Image>("left_image", 10);
  right_image_pub_ = create_publisher<sensor_msgs::msg::Image>("right_image", 10);

  // Use a timer to schedule periodic message publishing.
  // timer_ = create_wall_timer(1s, [this]() {return this->on_timer();});

  // Initialize MTC library and connect to cameras
  init_mtc();

  // while node is running, process frames
  bool ready;
  while (rclcpp::ok()) {
    mtc::Markers_IsBackgroundFrameProcessedGet(&ready);
    // Sleep for 1ms if not ready
    // if (!ready) {
    // std::this_thread::sleep_for(10ms);
    // }
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
  std::optional<std::string> calibrationDir = getMTHome();
  if (!calibrationDir) {
    RCLCPP_ERROR(this->get_logger(), "MTHome environment variable not set");
    return;
  }

  std::string markerDir = fs::path(*calibrationDir) / "Markers";
  std::string calibrationDirPath = fs::path(*calibrationDir) / "CalibrationFiles";

  MTC(mtc::Cameras_AttachAvailableCameras(calibrationDirPath.c_str()));
  if (mtc::Cameras_Count() < 1) {
    RCLCPP_ERROR(this->get_logger(), "No camera found!");
    return;
  }

  MTC(mtc::Cameras_ItemGet(0, &CurrCamera));
  MTC(mtc::Camera_SerialNumberGet(CurrCamera, &CurrCameraSerialNum));
  RCLCPP_INFO(this->get_logger(), "Attached %d camera(s). Curr camera is %d", mtc::Cameras_Count(),
              CurrCameraSerialNum);

  mtc::mtStreamingModeStruct mode{
    mtc::mtFrameType::Alternating,
    mtc::mtDecimation::Dec41,
    mtc::mtBitDepth::Bpp12};

  MTC(mtc::Cameras_StreamingModeSet(mode, CurrCameraSerialNum));

  int x, y;
  MTC(mtc::Camera_ResolutionGet(CurrCamera, &x, &y));
  RCLCPP_INFO(this->get_logger(), "The camera resolution is %d x %d", x, y);


  IsBackGroundProcessingEnabled = false;
  if (IsBackGroundProcessingEnabled) {
    MTC(mtc::Markers_BackGroundProcessSet(true));
    RCLCPP_INFO(this->get_logger(), "Background processing enabled");
  }

  MTC(mtc::Markers_LoadTemplates(const_cast<char *>(markerDir.c_str())));
  RCLCPP_INFO(this->get_logger(), "Loaded %d marker templates", mtc::Markers_TemplatesCount());

  IdentifiedMarkers = mtc::Collection_New();
  PoseXf = mtc::Xform3D_New();
}

void MicronTrackerDriver::on_timer()
{
  // Process frames and obtain measurements
  process_frames();
}

void MicronTrackerDriver::process_frames()
{
  auto msg = std::make_unique<visualization_msgs::msg::MarkerArray>();

  if (IsBackGroundProcessingEnabled) {
    mtc::Markers_GetIdentifiedMarkersFromBackgroundThread(CurrCamera);
  } else {
    MTC(mtc::Cameras_GrabFrame(0));
    MTC(mtc::Markers_ProcessFrame(0));
  }

  double frame_secs;
  mtc::Camera_FrameMTTimeSecsGet(CurrCamera, &frame_secs);
  auto frame_stamp = [frame_secs]() {
      auto sec =
        std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(frame_secs));
      auto nsec =
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(
        frame_secs - sec.count()));
      return rclcpp::Time(sec.count(), nsec.count());
    }();

  int x, y;
  MTC(mtc::Camera_ResolutionGet(CurrCamera, &x, &y));
  x /= 4;
  y /= 4;
  int QuarterSizeImageBufferSize = (x * y) * 3;

  std::vector<unsigned char> leftImageBuffer(QuarterSizeImageBufferSize);
  std::vector<unsigned char> rightImageBuffer(QuarterSizeImageBufferSize);
  mtc::Camera_24BitQuarterSizeImagesGet(CurrCamera, leftImageBuffer.data(),
      rightImageBuffer.data());

  // Publish left image
  auto left_image_msg = std::make_unique<sensor_msgs::msg::Image>();
  left_image_msg->header.frame_id = "camera";
  left_image_msg->header.stamp = frame_stamp;
  left_image_msg->height = y;
  left_image_msg->width = x;
  left_image_msg->encoding = "rgb8";
  left_image_msg->is_bigendian = false;
  left_image_msg->step = 3 * x;
  left_image_msg->data = std::move(leftImageBuffer);
  left_image_pub_->publish(std::move(left_image_msg));

  // Publish right image
  auto right_image_msg = std::make_unique<sensor_msgs::msg::Image>();
  right_image_msg->header.frame_id = "camera";
  right_image_msg->header.stamp = frame_stamp;
  right_image_msg->height = y;
  right_image_msg->width = x;
  right_image_msg->encoding = "rgb8";
  right_image_msg->is_bigendian = false;
  right_image_msg->step = 3 * x;
  right_image_msg->data = std::move(rightImageBuffer);
  right_image_pub_->publish(std::move(right_image_msg));

  MTC(mtc::Markers_IdentifiedMarkersGet(0, IdentifiedMarkers));
  auto clock = this->get_clock();
  RCLCPP_INFO_THROTTLE(this->get_logger(), *clock, 1000, "identified %d marker(s)",
                       mtc::Collection_Count(IdentifiedMarkers));

  for (int j = 1; j <= mtc::Collection_Count(IdentifiedMarkers); j++) {
    mtc::mtHandle Marker = mtc::Collection_Int(IdentifiedMarkers, j);
    MTC(mtc::Marker_Marker2CameraXfGet(Marker, CurrCamera, PoseXf, &IdentifyingCamera));

    if (IdentifyingCamera != 0) {
      char MarkerName[MT_MAX_STRING_LENGTH];
      double Position[3], Angle[3];
      // mtc::mtMeasurementHazardCode Hazard;

      MTC(mtc::Marker_NameGet(Marker, MarkerName, MT_MAX_STRING_LENGTH, 0));
      MTC(mtc::Xform3D_ShiftGet(PoseXf, Position));
      MTC(mtc::Xform3D_RotAnglesDegsGet(PoseXf, &Angle[0], &Angle[1], &Angle[2]));
      // MTC(mtc::Xform3D_HazardCodeGet(PoseXf, &Hazard));

      visualization_msgs::msg::Marker marker;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.id = j;
      marker.header.frame_id = "camera";
      marker.header.stamp = frame_stamp;
      marker.text = MarkerName;
      marker.pose.position.x = Position[0] / 1000;
      marker.pose.position.y = Position[1] / 1000;
      marker.pose.position.z = Position[2] / 1000;
      marker.pose.orientation.x = Angle[0];
      marker.pose.orientation.y = Angle[1];
      marker.pose.orientation.z = Angle[2];
      marker.pose.orientation.w = 1.0;
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

std::optional<std::string> getMTHome()
{
  if (const char * localNamePtr = getenv("MTHome")) {
    return std::string(localNamePtr);
  }
  return std::nullopt;
}

}  // namespace microntracker_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(microntracker_components::MicronTrackerDriver)
