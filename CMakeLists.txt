cmake_minimum_required(VERSION 2.8)

project(catkin)

set(catkin_EXTRAS_DIR_VALUE ${catkin_SOURCE_DIR}/cmake)
set(catkin_EXTRAS_DIR ${catkin_EXTRAS_DIR_VALUE} CACHE FILEPATH "catkin extras")

include(${catkin_EXTRAS_DIR}/all.cmake)

install_cmake_infrastructure(catkin
  VERSION 0.0.7
  )

install_cmake_config_version(ROS 12.04)

configure_file(ros-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/ros-config.cmake
  @ONLY)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/ros-config.cmake
  DESTINATION share/cmake/ROS)

install(DIRECTORY cmake/
  DESTINATION share/cmake/catkin
  USE_SOURCE_PERMISSIONS
  )

configure_file(setup.sh.in ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/setup.sh)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/setup.sh
  setup.bash
  setup.zsh
  DESTINATION ${CMAKE_INSTALL_PREFIX}
  )

catkin_package(catkin)
