cmake_minimum_required(VERSION 2.8)
project(d)

find_package(catkin)
find_package(c)
find_package(b)
find_package(a)
include_directories(${b_INCLUDE_DIRS})

include_directories(${c_INCLUDE_DIRS})

add_library(d SHARED lib.cpp)
target_link_libraries(d ${b_LIBRARIES} ${c_LIBRARIES})

add_message_files(
  DIRECTORY msg
  FILES DMsg.msg
)

catkin_project(d
  INCLUDE_DIRS include
  LIBRARIES d
  )

# catkin_python_setup()
# enable_python(d)

# generate_messages(DEPENDENCIES std_msgs b c a)

