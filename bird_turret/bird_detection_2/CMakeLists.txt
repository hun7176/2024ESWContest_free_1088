cmake_minimum_required(VERSION 3.10)
project(bird_detection_2)

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
  src/detection_2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

