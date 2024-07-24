cmake_minimum_required(VERSION 3.0.2)
project(camera_launch)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs image_transport
)

# Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

