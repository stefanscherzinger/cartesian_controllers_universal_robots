cmake_minimum_required(VERSION 3.8)
project(cartesian_controllers_universal_robots)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    etc
    launch
    urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
