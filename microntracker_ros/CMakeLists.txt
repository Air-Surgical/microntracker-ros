cmake_minimum_required(VERSION 3.8)
project(microntracker_ros NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
