// Copyright 2025 Air Surgical, Inc.

#include <chrono>
#include <filesystem>
#include <map>
#include <string>

#include "microntracker_components/microntracker_component.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace microntracker_components
{

MicronTrackerDriver::MicronTrackerDriver(const rclcpp::NodeOptions & options)
: Node("microntracker_driver", options), count_(0)
{
  marker_array_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
  image_left_pub_ = create_publisher<sensor_msgs::msg::Image>("left/image_raw", 10);
  image_right_pub_ = create_publisher<sensor_msgs::msg::Image>("right/image_raw", 10);

  param_listener_ = std::make_shared<ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Initialize MTC library and connect to cameras
  init_mtc();
  is_alive = true;
  process_thread = std::thread([this](){
        while (is_alive) {
          process_frames();
        }
  });
}

MicronTrackerDriver::~MicronTrackerDriver()
{
  is_alive = false;
  process_thread.join();
  // FIXME: this sigfaults teardown
  // mtc::Cameras_Detach();
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
  if (IsBackGroundProcessingEnabled) {
    mtc::Markers_GetIdentifiedMarkersFromBackgroundThread(CurrCamera);
  } else {
    MTR(mtc::Cameras_GrabFrame(0));
    MTR(mtc::Markers_ProcessFrame(0));
  }

  double frame_secs;
  MTR(mtc::Camera_FrameMTTimeSecsGet(CurrCamera, &frame_secs));
  auto stamp = [this, frame_secs]() {
      auto duration = rclcpp::Duration::from_seconds(frame_secs);
      return this->mt_epoch + duration;
    }();
  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = params_.frame_id;

  publish_images(header);
  publish_markers(header);
}

void MicronTrackerDriver::publish_markers(const std_msgs::msg::Header & header)
{
  auto msg = std::make_unique<visualization_msgs::msg::MarkerArray>();
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
      std::string MarkerName(MT_MAX_STRING_LENGTH, '\0');
      // mtc::mtMeasurementHazardCode Hazard;

      MTR(mtc::Marker_NameGet(Marker, MarkerName.data(), MT_MAX_STRING_LENGTH, 0));
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

      // Publish the transform
      geometry_msgs::msg::TransformStamped transform;
      transform.header = header;
      transform.child_frame_id = MarkerName;
      transform.transform.translation.x = position[0] / 1000;
      transform.transform.translation.y = position[1] / 1000;
      transform.transform.translation.z = position[2] / 1000;
      transform.transform.rotation.x = orientation[0];
      transform.transform.rotation.y = orientation[1];
      transform.transform.rotation.z = orientation[2];
      transform.transform.rotation.w = orientation[3];
      tf_broadcaster_->sendTransform(transform);
    }
  }
  marker_array_pub_->publish(std::move(msg));
}

void MicronTrackerDriver::publish_images(const std_msgs::msg::Header & header)
{
  struct DecimationParams
  {
    int decimation;
    mtc::mtCompletionCode (*images_get)(mtc::mtHandle, unsigned char *, unsigned char *);
  };

  std::string encoding = params_.encoding;
  const std::map<mtc::mtDecimation, DecimationParams> decimation_map = {
    {mtc::mtDecimation::Dec11,
      {1, encoding ==
        "rgb8" ? mtc::Camera_24BitImagesGet : mtc::Camera_ImagesGet}},
    {mtc::mtDecimation::Dec21,
      {2, encoding ==
        "rgb8" ? mtc::Camera_24BitHalfSizeImagesGet : mtc::Camera_HalfSizeImagesGet}},
    {mtc::mtDecimation::Dec41,
      {4, encoding ==
        "rgb8" ? mtc::Camera_24BitQuarterSizeImagesGet : mtc::Camera_QuarterSizeImagesGet}}
  };

  mtc::mtStreamingModeStruct mode;
  MTR(mtc::Camera_StreamingModeGet(CurrCamera, &mode));
  auto it = decimation_map.find(mode.decimation);
  if (it == decimation_map.end()) {
    RCLCPP_ERROR(this->get_logger(), "Invalid decimation mode");
    return;
  }
  const auto & params = it->second;

  int width, height, step, depth;
  MTR(mtc::Camera_ResolutionGet(CurrCamera, &width, &height));
  width /= params.decimation;
  height /= params.decimation;

  depth = encoding == "rgb8" ? 3 : 2;
  step = width * depth;
  int image_buffer_size = (width * height) * depth;

  std::vector<unsigned char> image_left_data(image_buffer_size);
  std::vector<unsigned char> image_right_data(image_buffer_size);
  MTR(params.images_get(CurrCamera, image_left_data.data(), image_right_data.data()));

  auto publish_image = [&](
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & publisher,
    const std::vector<unsigned char> & data) {
      auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
      image_msg->header = header;
      image_msg->height = height;
      image_msg->width = width;
      image_msg->encoding = encoding;
      image_msg->is_bigendian = false;
      image_msg->step = step;
      image_msg->data = data;
      publisher->publish(std::move(image_msg));
    };
  publish_image(image_left_pub_, image_left_data);
  publish_image(image_right_pub_, image_right_data);
}

}  // namespace microntracker_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(microntracker_components::MicronTrackerDriver)
