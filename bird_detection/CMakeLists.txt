cmake_minimum_required(VERSION 3.10)
project(bird_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/basic.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

