#
#  TOPLEVEL cmakelists
#
cmake_minimum_required(VERSION 2.8)

# set(CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})
# set(CATKIN_LANGS cpp)
# set(CATKIN_LOG 9)

if (IS_DIRECTORY ${CMAKE_SOURCE_DIR}/catkin)
  message("+++ catkin")
  add_subdirectory(catkin)
else()
  find_package(catkin)
endif()

return()
catkin_workspace()

return()
message("+++ genmsg")
add_subdirectory(genmsg)

foreach (l ${CATKIN_LANGS})
  message("+++ gen${l}")
  add_subdirectory(gen${l})
endforeach()

set(CATKIN TRUE CACHE BOOL "Yeah catkin! yay!")
# ROS_LANGS should get detected and set in subsequent steps
find_package(catkin)

