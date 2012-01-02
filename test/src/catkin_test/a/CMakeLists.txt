cmake_minimum_required(VERSION 2.8)
project(a)
find_package(catkin)

add_library(a SHARED lib.cpp)

add_message_files(
  DIRECTORY msg
  FILES AMsg.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_project(a
  VERSION 0.0.1
  INCLUDE_DIRS include
  LIBRARIES a
  PYTHONPATH src
  )

enable_python(a)
