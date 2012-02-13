cmake_minimum_required(VERSION 2.8)

project(catkin)

set(catkin_EXTRAS_DIR_VALUE ${catkin_SOURCE_DIR}/cmake)
set(catkin_EXTRAS_DIR ${catkin_EXTRAS_DIR_VALUE} CACHE FILEPATH "catkin extras")

include(${catkin_EXTRAS_DIR}/all.cmake)

list(APPEND CMAKE_MODULE_PATH ${catkin_EXTRAS_DIR}/Modules)

add_subdirectory(env-hooks)
add_subdirectory(bin)
catkin_generic_hooks()

catkin_project(catkin)

install_cmake_config_version(ROS 12.04)

configure_file(ros-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/ros-config.cmake
  @ONLY)

configure_file(ros-config.buildspace.cmake.in
  ${CMAKE_BINARY_DIR}/cmake/ros/ros-config.cmake
  @ONLY)

#EAR: share/<name>/cmake is our convention?
#according to the man page of cmake, find_package searches:
# <prefix>/(lib/<arch>|lib|share)/cmake/<name>*/          (U)
# <prefix>/(lib/<arch>|lib|share)/<name>*/                (U)
# <prefix>/(lib/<arch>|lib|share)/<name>*/(cmake|CMake)/  (U)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/ros-config.cmake
  DESTINATION share/ros/cmake)

install(DIRECTORY cmake
  #to be consistent, all catkin specific cmake should live under catkin.
  #NOTE the trailing slash here.  You need it.
  DESTINATION share/catkin/
  USE_SOURCE_PERMISSIONS
  )

#HACK so that eigen can be found if catkin is in the
#workspace... this is very bad that catkin knows about eigen...
file(COPY cmake/Modules/eigen-config.cmake
  DESTINATION ${CMAKE_BINARY_DIR}/cmake
)
catkin_export_python()

stamp(cmake/stack_get.py)

catkin_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc ${CMAKE_CURRENT_BINARY_DIR}/doc)

install(FILES stack.yaml DESTINATION share/catkin)
