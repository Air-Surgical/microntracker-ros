cmake_minimum_required(VERSION 3.8)
project(microntracker_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(microntracker REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

generate_parameter_library(
  microntracker_components_parameters
  src/microntracker_components_parameters.yaml
)

add_library(microntracker_component SHARED src/microntracker_component.cpp)
target_include_directories(microntracker_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(microntracker_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(microntracker_component
  microntracker
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2_ros
  visualization_msgs
)
target_link_libraries(microntracker_component
  microntracker_components_parameters
  ${microntracker_LIBRARIES}
)

rclcpp_components_register_node(
    microntracker_component
    PLUGIN "microntracker_components::MicronTrackerDriver"
    EXECUTABLE microntracker_driver
)

ament_export_targets(export_microntracker_component)
install(TARGETS microntracker_component microntracker_components_parameters
        EXPORT export_microntracker_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
