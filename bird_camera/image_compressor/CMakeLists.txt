cmake_minimum_required(VERSION 3.0.2)
project(image_compressor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs cv_bridge std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 이 패키지는 현재 C++ 코드가 없지만, C++ 노드를 추가할 경우 이 섹션이 필요할 수 있습니다.
# add_executable(image_compressor_node src/image_compressor_node.cpp)
# target_link_libraries(image_compressor_node ${catkin_LIBRARIES})

