cmake_minimum_required(VERSION 2.8)
project(c)

find_package(catkin)
find_package(a)
include_directories(${a_INCLUDE_DIRS})

add_library(c SHARED lib.cpp)
target_link_libraries(c ${a_LIBRARIES})

generate_msgs(c
  PATH msg
  MESSAGES msg/CMsg.msg
  DEPENDENCIES std_msgs a
  )

install_cmake_infrastructure(c
  VERSION 0.0.1
  INCLUDE_DIRS include
  LIBRARIES c
  MSG_DIRS msg
  )

enable_python(c)

