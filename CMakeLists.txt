cmake_minimum_required(VERSION 3.0.2)
project(rgb_mapping)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  tf
  pcl_ros
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.5 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_tf
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(rgb_mapping src/rgb_mapping.cpp)
target_link_libraries(rgb_mapping ${catkin_LIBRARIES})
