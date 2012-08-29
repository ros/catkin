#
#  TOPLEVEL cmakelists
#
cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0011 NEW)

include(${CMAKE_SOURCE_DIR}/workspace-config.cmake OPTIONAL)

set(CATKIN YES)
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/cmake)

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if (IS_DIRECTORY ${CMAKE_SOURCE_DIR}/catkin)
  message(STATUS "+++ catkin")
  set(CATKIN_BUILD_STACKS "ALL" CACHE STRING
    "List of ';' separated stacks to build, or ALL for all.")
  set(CATKIN_BLACKLIST_STACKS "NONE" CACHE STRING
    "List of ';' separated stacks to completely exclude from cmake traversal.")
  add_subdirectory(catkin)
else()
  find_package(catkin)
endif()

catkin_workspace()
