cmake_minimum_required(VERSION 3.8)
project(microntracker_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
# find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)

# include_directories(include)

# # create ament index resource which references the libraries in the binary dir
# set(node_plugins "")

# add_library(microntracker_component SHARED
#   src/microntracker_component.cpp)
# target_compile_definitions(microntracker_component
#   PRIVATE "MICRONTRACKER_COMPONENTS_BUILDING_DLL")
# target_link_libraries(microntracker_component PUBLIC
#   rclcpp::rclcpp
#   rclcpp_components::component
#   ${std_msgs_TARGETS})
# rclcpp_components_register_nodes(microntracker_component
#   "microntracker_components::MicronTrackerDriver"
#   EXECUTABLE microntracker_driver)
# set(node_plugins "${node_plugins}microntracker_components::MicronTrackerDriver;$<TARGET_FILE:microntracker_component>\n")

# # since the package installs libraries without exporting them
# # it needs to make sure that the library path is being exported
# if(NOT WIN32)
#   ament_environment_hooks(
#     "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
# endif()

# install(TARGETS
#   microntracker_component
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)

add_library(microntracker_component SHARED src/microntracker_component.cpp)
target_include_directories(microntracker_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(microntracker_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(microntracker_component
  rclcpp
  rclcpp_components
  std_msgs
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
