// Copyright 2025 Air Surgical, Inc.

#include <array>
#include <chrono>
#include <filesystem>
#include <map>
#include <memory>
#include <string>

#include "microntracker_components/microntracker_component.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace microntracker_components
{

MicronTrackerDriver::MicronTrackerDriver(const rclcpp::NodeOptions & options)
: Node("microntracker_driver", options), count_(0)
{
  camera_info_left_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 10);
  camera_info_right_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 10);
  image_left_pub_ = create_publisher<sensor_msgs::msg::Image>("left/image_raw", 10);
  image_right_pub_ = create_publisher<sensor_msgs::msg::Image>("right/image_raw", 10);
  marker_array_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);

  param_listener_ = std::make_shared<ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Initialize MTC library and connect to cameras
  init_mtc();
  init_info();
  is_alive = true;
  process_thread = std::thread([this](){
        while (is_alive) {
          process_frame();
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

void MicronTrackerDriver::init_info()
{
  int width, height;
  MTR(mtc::Camera_ResolutionGet(CurrCamera, &width, &height));

  int rows = height / 8;
  int cols = width / 8;
  float xSize = 4 * 20.0f;
  float ySize = 3 * 20.0f;
  float zDepth = 1000.0f;
  auto objectPoints = generateObjectPoints(rows, cols, xSize, ySize, zDepth);
  auto imagePoints1 = std::vector<std::vector<cv::Point2f>>(0);
  auto imagePoints2 = std::vector<std::vector<cv::Point2f>>(0);

  for (const auto & view : objectPoints) {
    std::vector<cv::Point2f> projectedPointsLeft;
    std::vector<cv::Point2f> projectedPointsRight;

    for (const auto & point : view) {
      // Convert cv::Point3f to std::array<double, 3> for mtc::Camera_ProjectionOnImage
      std::array<double, 3> xyz = {point.x, point.y, point.z};

      // Create unique pointers for the output x and y coordinates
      double l_outX, l_outY, r_outX, r_outY;

      // Project the point onto the left image plane
      auto rcLeft = mtc::Camera_ProjectionOnImage(CurrCamera, mtr::mtSideI::mtLeft, xyz.data(),
          &l_outX,
          &l_outY);
      if (rcLeft != mtc::mtOK) {
        RCLCPP_ERROR_ONCE(this->get_logger(), "Projection on left image failed: %d", rcLeft);
      }
      projectedPointsLeft.emplace_back(static_cast<float>(l_outX), static_cast<float>(l_outY));

      // Project the point onto the right image plane
      auto rcRight = mtc::Camera_ProjectionOnImage(CurrCamera, mtr::mtSideI::mtRight, xyz.data(),
          &r_outX,
          &r_outY);
      if (rcRight != mtc::mtOK) {
        RCLCPP_ERROR_ONCE(this->get_logger(), "Projection on right image failed: %d", rcRight);
      }
      projectedPointsRight.emplace_back(static_cast<float>(r_outX), static_cast<float>(r_outY));
    }

    // Add the projected points for this view to the respective image points vectors
    imagePoints1.push_back(projectedPointsLeft);
    imagePoints2.push_back(projectedPointsRight);
  }

  cv::Size imageSize(width, height);
  auto createCameraMatrix = [imageSize](double focalLength)
    {
      cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
      cameraMatrix.at<double>(0, 0) = focalLength;  // fx
      cameraMatrix.at<double>(1, 1) = focalLength;  // fy
      cameraMatrix.at<double>(0, 2) = imageSize.width / 2.0;  // cx
      cameraMatrix.at<double>(1, 2) = imageSize.height / 2.0;  // cy
      return cameraMatrix;
    };
  cv::Mat cameraMatrix1 = createCameraMatrix(1500);
  cv::Mat cameraMatrix2 = createCameraMatrix(1500);
  cv::Mat distCoeffs1 = cv::Mat::zeros(5, 1, CV_64F);
  cv::Mat distCoeffs2 = cv::Mat::zeros(5, 1, CV_64F);
  cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat T = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat E = cv::Mat::zeros(3, 3, CV_64F);
  cv::Mat F = cv::Mat::zeros(3, 3, CV_64F);
  cv::Mat perViewErrors;
  int flags =
    cv::CALIB_FIX_ASPECT_RATIO |
    cv::CALIB_FIX_K3 |
    cv::CALIB_USE_INTRINSIC_GUESS |
    cv::CALIB_ZERO_TANGENT_DIST;
  cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 1e-6);

  cv::stereoCalibrate(
      objectPoints,                             // 3D points
      imagePoints1,                             // 2D points for left camera
      imagePoints2,                             // 2D points for right camera
      cameraMatrix1,                            // camera matrix for left camera
      distCoeffs1,                              // distortion coefficients for left camera
      cameraMatrix2,                            // camera matrix for right camera
      distCoeffs2,                              // distortion coefficients for right camera
      imageSize,                                // image size
      R,                                        // rotation matrix
      T,                                        // translation vector
      E,                                        // essential matrix
      F,                                        // fundamental matrix
      perViewErrors,                            // per-view reprojection errors
      flags,                                    // calibration flags
      criteria                                  // termination criteria
  );

  // Initialize camera info
  sensor_msgs::msg::CameraInfo camera_info_left;
  camera_info_left.width = imageSize.width;
  camera_info_left.height = imageSize.height;
  camera_info_left.distortion_model = "plumb_bob";
  camera_info_left.header.stamp = this->now();
  camera_info_left.header.frame_id = params_.frame_id;
  camera_info_left.binning_x = 4;
  camera_info_left.binning_y = 4;
  sensor_msgs::msg::CameraInfo camera_info_right = camera_info_left;

  auto set_camera_info = [](sensor_msgs::msg::CameraInfo & camera_info, const cv::Mat & M,
    const cv::Mat & D, const cv::Mat & R) {
      camera_info.d = {
        D.at<double>(0),
        D.at<double>(1),
        D.at<double>(2),
        D.at<double>(3),
        D.at<double>(4)};
      camera_info.k = {
        M.at<double>(0, 0), M.at<double>(0, 1), M.at<double>(0, 2),
        M.at<double>(1, 0), M.at<double>(1, 1), M.at<double>(1, 2),
        M.at<double>(2, 0), M.at<double>(2, 1), M.at<double>(2, 2)};
      camera_info.r = {
        R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2)};
      camera_info.p = {
        M.at<double>(0, 0), M.at<double>(0, 1), M.at<double>(0, 2), 0.0,
        M.at<double>(1, 0), M.at<double>(1, 1), M.at<double>(1, 2), 0.0,
        M.at<double>(2, 0), M.at<double>(2, 1), M.at<double>(2, 2), 0.0};
    };

  // Set camera info for left and right cameras
  set_camera_info(camera_info_left, cameraMatrix1, distCoeffs1, R);
  set_camera_info(camera_info_right, cameraMatrix2, distCoeffs2, R);

  // Publish the camera info
  camera_info_left_pub_->publish(camera_info_left);
  camera_info_right_pub_->publish(camera_info_right);
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

  MTR(mtc::Markers_LoadTemplates(const_cast<char *>(markerDir.c_str())));
  RCLCPP_INFO(this->get_logger(), "Loaded %d marker templates", mtc::Markers_TemplatesCount());

  IdentifiedMarkers = mtc::Collection_New();
  PoseXf = mtc::Xform3D_New();
}

void MicronTrackerDriver::process_frame()
{
  MTR(mtc::Cameras_GrabFrame(0));
  MTR(mtc::Markers_ProcessFrame(0));

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
  std::vector<double> tooltip_position(3);
  std::vector<double> tooltip_orientation(4);

  for (int j = 1; j <= mtc::Collection_Count(IdentifiedMarkers); j++) {
    mtc::mtHandle Marker = mtc::Collection_Int(IdentifiedMarkers, j);
    MTR(mtc::Marker_Marker2CameraXfGet(Marker, CurrCamera, PoseXf, &IdentifyingCamera));

    if (IdentifyingCamera != 0) {
      std::string MarkerName(MT_MAX_STRING_LENGTH, '\0');
      // mtc::mtMeasurementHazardCode Hazard;

      MTR(mtc::Marker_NameGet(Marker, MarkerName.data(), MT_MAX_STRING_LENGTH, 0));
      MarkerName.resize(std::strlen(MarkerName.c_str()));

      MTR(mtc::Xform3D_ShiftGet(PoseXf, position.data()));
      // TODO(@ruffsl): why do we need this inverse?
      MTR(mtc::Xform3D_Inverse(PoseXf, PoseXf));
      MTR(mtc::Xform3D_RotQuaternionsGet(PoseXf, orientation.data()));
      // MTR(mtc::Xform3D_HazardCodeGet(PoseXf, &Hazard));

      auto PoseXf_Tooltip = mtc::Xform3D_New();
      MTR(mtc::Marker_Tooltip2MarkerXfGet(Marker, PoseXf_Tooltip));
      MTR(mtc::Xform3D_ShiftGet(PoseXf_Tooltip, tooltip_position.data()));
      MTR(mtc::Xform3D_Inverse(PoseXf_Tooltip, PoseXf_Tooltip));
      MTR(mtc::Xform3D_RotQuaternionsGet(PoseXf_Tooltip, tooltip_orientation.data()));
      mtc::Xform3D_Free(PoseXf_Tooltip);

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
      geometry_msgs::msg::TransformStamped marker_tfs;
      marker_tfs.header = header;
      marker_tfs.child_frame_id = MarkerName;
      marker_tfs.transform.translation.x = position[0] / 1000;
      marker_tfs.transform.translation.y = position[1] / 1000;
      marker_tfs.transform.translation.z = position[2] / 1000;
      marker_tfs.transform.rotation.x = orientation[0];
      marker_tfs.transform.rotation.y = orientation[1];
      marker_tfs.transform.rotation.z = orientation[2];
      marker_tfs.transform.rotation.w = orientation[3];
      tf_broadcaster_->sendTransform(marker_tfs);

      geometry_msgs::msg::TransformStamped tooltip_tfs;
      tooltip_tfs.header = header;
      tooltip_tfs.header.frame_id = MarkerName;
      tooltip_tfs.child_frame_id = MarkerName + "_tooltip";
      tooltip_tfs.transform.translation.x = tooltip_position[0] / 1000;
      tooltip_tfs.transform.translation.y = tooltip_position[1] / 1000;
      tooltip_tfs.transform.translation.z = tooltip_position[2] / 1000;
      tooltip_tfs.transform.rotation.x = tooltip_orientation[0];
      tooltip_tfs.transform.rotation.y = tooltip_orientation[1];
      tooltip_tfs.transform.rotation.z = tooltip_orientation[2];
      tooltip_tfs.transform.rotation.w = tooltip_orientation[3];
      tf_broadcaster_->sendTransform(tooltip_tfs);
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

  depth = encoding == "rgb8" ? 3 : 3;
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

std::vector<std::vector<cv::Point3f>> generateObjectPoints(
  int rows, int cols, float xSize, float ySize, float zDepth)
{
  std::vector<std::vector<cv::Point3f>> objectPoints(1);  // Single view
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      // Generate points centered around the x-y axis
      float x = (j - cols / 2.0f) * xSize;
      float y = (i - rows / 2.0f) * ySize;
      objectPoints[0].emplace_back(x, y, zDepth);
    }
  }
  return objectPoints;
}

}  // namespace microntracker_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(microntracker_components::MicronTrackerDriver)
