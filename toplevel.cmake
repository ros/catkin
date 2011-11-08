cmake_minimum_required(VERSION 2.8)

set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_BINARY_DIR})
set(ROSBUILD_LANGS cpp)
# set(ROSBUILD_LOG 9)
add_custom_target(debs)
add_subdirectory(rosbuild)
add_subdirectory(genmsg)

foreach (l ${ROSBUILD_LANGS})
  add_subdirectory(gen${l})
endforeach()

set(ROSBUILD TRUE CACHE BOOL "Yeah rosbuild! yay!")
# ROS_LANGS should get detected and set in subsequent steps
find_package(rosbuild)

wgbuild_workspace()

