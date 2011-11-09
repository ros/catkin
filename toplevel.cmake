cmake_minimum_required(VERSION 2.8)

set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_BINARY_DIR})
set(CATKIN_LANGS cpp)
# set(CATKIN_LOG 9)
add_custom_target(debs)
add_subdirectory(catkin)
add_subdirectory(genmsg)

foreach (l ${CATKIN_LANGS})
  add_subdirectory(gen${l})
endforeach()

set(CATKIN TRUE CACHE BOOL "Yeah catkin! yay!")
# ROS_LANGS should get detected and set in subsequent steps
find_package(catkin)

catkin_workspace()

