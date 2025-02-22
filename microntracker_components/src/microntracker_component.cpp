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
#include "std_msgs/msg/string.hpp"

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
  // Create a publisher of "std_mgs/String" messages on the "chatter" topic.
  pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);

  // Use a timer to schedule periodic message publishing.
  timer_ = create_wall_timer(1s, [this]() {return this->on_timer();});

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
    mtc::mtBitDepth::Bpp14};

  MTC(mtc::Cameras_StreamingModeSet(mode, CurrCameraSerialNum));

  int x, y;
  MTC(mtc::Camera_ResolutionGet(CurrCamera, &x, &y));
  RCLCPP_INFO(this->get_logger(), "The camera resolution is %d x %d", x, y);

  MTC(mtc::Markers_LoadTemplates(const_cast<char *>(markerDir.c_str())));
  RCLCPP_INFO(this->get_logger(), "Loaded %d marker templates", mtc::Markers_TemplatesCount());
}

void MicronTrackerDriver::on_timer()
{
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = "Hello World: " + std::to_string(++count_);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg->data.c_str());
  std::flush(std::cout);

  // Put the message into a queue to be processed by the middleware.
  // This call is non-blocking.
  pub_->publish(std::move(msg));

  // Process frames and obtain measurements
  process_frames();
}

void MicronTrackerDriver::process_frames()
{
  mtc::mtHandle IdentifiedMarkers = mtc::Collection_New();
  mtc::mtHandle PoseXf = mtc::Xform3D_New();

  for (int i = 0; i < 20; i++) {
    MTC(mtc::Cameras_GrabFrame(0));
    MTC(mtc::Markers_ProcessFrame(0));

    if (i < 10) {continue;}

    MTC(mtc::Markers_IdentifiedMarkersGet(0, IdentifiedMarkers));
    RCLCPP_INFO(this->get_logger(), "%d: identified %d marker(s)", i,
        mtc::Collection_Count(IdentifiedMarkers));

    for (int j = 1; j <= mtc::Collection_Count(IdentifiedMarkers); j++) {
      mtc::mtHandle Marker = mtc::Collection_Int(IdentifiedMarkers, j);
      MTC(mtc::Marker_Marker2CameraXfGet(Marker, CurrCamera, PoseXf, &IdentifyingCamera));

      if (IdentifyingCamera != 0) {
        char MarkerName[MT_MAX_STRING_LENGTH];
        double Position[3], Angle[3];
        mtc::mtMeasurementHazardCode Hazard;

        MTC(mtc::Marker_NameGet(Marker, MarkerName, MT_MAX_STRING_LENGTH, 0));
        MTC(mtc::Xform3D_ShiftGet(PoseXf, Position));
        MTC(mtc::Xform3D_RotAnglesDegsGet(PoseXf, &Angle[0], &Angle[1], &Angle[2]));
        MTC(mtc::Xform3D_HazardCodeGet(PoseXf, &Hazard));

        RCLCPP_INFO(this->get_logger(),
            ">> %s at (%0.2f, %0.2f, %0.2f), rotation (degs): (%0.1f, %0.1f, %0.1f) %s",
                    MarkerName, Position[0], Position[1], Position[2], Angle[0], Angle[1], Angle[2],
            mtc::MTHazardCodeString(Hazard));
      }
    }
  }

  mtc::Collection_Free(IdentifiedMarkers);
  mtc::Xform3D_Free(PoseXf);
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
