cmake_minimum_required(VERSION 3.8)
project(microntracker_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(microntracker REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# Specify the paths to the required libraries
set(MICRONTRACKER_LIB_DIR "/opt/air_ws/install/microntracker/lib")
link_directories(${MICRONTRACKER_LIB_DIR})

add_library(microntracker_component SHARED src/microntracker_component.cpp)
target_include_directories(microntracker_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(microntracker_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(microntracker_component
  microntracker
  rclcpp
  rclcpp_components
  std_msgs
)
target_link_libraries(microntracker_component
  ${MICRONTRACKER_LIB_DIR}/libCLAllSerial_gcc48_v3_2.so
  ${MICRONTRACKER_LIB_DIR}/libCLProtocol_gcc48_v3_2.so
  ${MICRONTRACKER_LIB_DIR}/libFirmwareUpdate_gcc48_v3_2.so
  ${MICRONTRACKER_LIB_DIR}/libGCBase_gcc48_v3_2.so
  ${MICRONTRACKER_LIB_DIR}/libGenApi_gcc48_v3_2.so
  ${MICRONTRACKER_LIB_DIR}/libJadakApi-4.10.5.so
  ${MICRONTRACKER_LIB_DIR}/libLog_gcc48_v3_2.so
  ${MICRONTRACKER_LIB_DIR}/libMTC.so
  ${MICRONTRACKER_LIB_DIR}/libMathParser_gcc48_v3_2.so
  ${MICRONTRACKER_LIB_DIR}/libNodeMapData_gcc48_v3_2.so
  ${MICRONTRACKER_LIB_DIR}/libXmlParser_gcc48_v3_2.so
  ${MICRONTRACKER_LIB_DIR}/libZXing.a
  ${MICRONTRACKER_LIB_DIR}/liblog4cpp_gcc48_v3_2.so
)

rclcpp_components_register_node(
    microntracker_component
    PLUGIN "microntracker_components::MicronTrackerDriver"
    EXECUTABLE microntracker_driver
)

ament_export_targets(export_microntracker_component)
install(TARGETS microntracker_component
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
