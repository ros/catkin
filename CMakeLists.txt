cmake_minimum_required(VERSION 2.8)

project(catkin)

set(catkin_EXTRAS_DIR_VALUE ${catkin_SOURCE_DIR}/cmake)
set(catkin_EXTRAS_DIR ${catkin_EXTRAS_DIR_VALUE} CACHE FILEPATH "catkin extras")

include(${catkin_EXTRAS_DIR}/all.cmake)

catkin_project(catkin
  PYTHONPATH python
  )

install_cmake_config_version(ROS 12.04)

configure_file(ros-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/ros-config.cmake
  @ONLY)

configure_file(ros-config.buildspace.cmake.in
  ${CMAKE_BINARY_DIR}/cmake/ROS/ros-config.cmake
  @ONLY)

#EAR: share/<name>/cmake is our convention?
#according to the man page of cmake, find_package searches:
# <prefix>/(lib/<arch>|lib|share)/cmake/<name>*/          (U)
# <prefix>/(lib/<arch>|lib|share)/<name>*/                (U)
# <prefix>/(lib/<arch>|lib|share)/<name>*/(cmake|CMake)/  (U)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/ros-config.cmake
  DESTINATION share/ROS/cmake)

install(DIRECTORY cmake/
  DESTINATION share/catkin/cmake    #to be consistent, all catkin specific cmake should live under catkin.
  USE_SOURCE_PERMISSIONS
  )

configure_file(cmake/templates/setup.sh.installable.in
  ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/setup.sh)
configure_file(cmake/templates/env.sh.installable.in
  ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/env.sh)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/setup.sh
  cmake/templates/setup.bash
  cmake/templates/setup.zsh
  DESTINATION ${CMAKE_INSTALL_PREFIX}
  )

install(PROGRAMS
  ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/env.sh
  DESTINATION ${CMAKE_INSTALL_PREFIX}
  )

catkin_package(catkin)
enable_python(catkin)

stamp(cmake/catkin_generate_debian.py)
stamp(cmake/stack_get.py)

sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc ${CMAKE_CURRENT_BINARY_DIR}/doc)
