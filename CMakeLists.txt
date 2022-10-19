cmake_minimum_required(VERSION 3.10)
project(cartesian_controllers_ur_robots)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cartesian_controllers_ur_robots
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

# Install config files
install(DIRECTORY cfg etc launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
