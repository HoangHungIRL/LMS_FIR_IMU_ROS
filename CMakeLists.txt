cmake_minimum_required(VERSION 3.0.2)
project(fir_lms_imu_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  eigen_conversions
  message_filters
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs eigen_conversions message_filters
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(fir_lms_imu_node
  src/fir_lms_imu_node.cpp
)
target_link_libraries(fir_lms_imu_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
add_dependencies(fir_lms_imu_node ${catkin_EXPORTED_TARGETS})