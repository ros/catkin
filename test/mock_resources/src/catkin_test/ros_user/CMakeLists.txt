cmake_minimum_required(VERSION 2.8)

find_package(ROS 12.04 COMPONENTS
  cpp_common rostime roscpp_traits roscpp_serialization sensor_msgs)

include_directories(${ROS_INCLUDE_DIRS})

add_executable(something main.cpp)
target_link_libraries(something ${ROS_LIBRARIES})
