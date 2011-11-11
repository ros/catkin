cmake_minimum_required(VERSION 2.8)

project(catkin)

set(catkin_EXTRAS_DIR_VALUE ${catkin_SOURCE_DIR}/cmake)
set(catkin_EXTRAS_DIR ${catkin_EXTRAS_DIR_VALUE})

configure_file(cmake/catkin-config.cmake.in
  ${catkin_EXTRAS_DIR}/catkin-config.cmake
  @ONLY
  )

include(${catkin_EXTRAS_DIR}/catkin-config.cmake)

set(catkin_EXTRAS_DIR_VALUE ${CMAKE_INSTALL_PREFIX}/share/cmake/catkin)
configure_file(cmake/catkin-config.cmake.in
  cmake_install/catkin-config.cmake
  @ONLY
  )

install(
  DIRECTORY cmake/
  DESTINATION share/cmake/catkin
  )

install(
  FILES
  ${CMAKE_CURRENT_BINARY_DIR}/cmake_install/catkin-config.cmake
  debian.rules
  DESTINATION share/cmake/catkin
  COMPONENT catkin
  )

#include(${catkin_BINARY_DIR}/cmake/catkin-config.cmake)
#set(catkin_DIR ${catkin_BINARY_DIR}/cmake)
#catkin_package(catkin)