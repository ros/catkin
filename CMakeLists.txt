cmake_minimum_required(VERSION 2.8)

project(catkin)

set(catkin_EXTRAS_DIR_VALUE ${catkin_SOURCE_DIR}/cmake)
set(catkin_EXTRAS_DIR ${catkin_EXTRAS_DIR_VALUE} CACHE FILEPATH "catkin extras")

configure_file(cmake/tests.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/tests.cmake
  @ONLY)

include(${catkin_EXTRAS_DIR}/all.cmake)

list(APPEND CMAKE_MODULE_PATH ${catkin_EXTRAS_DIR}/Modules)

add_subdirectory(env-hooks)
add_subdirectory(bin)
catkin_generic_hooks()

# We undefine CATKIN_CURRENT_STACK here to avoid a spurious warning when
# CATKIN_CURRENT_STACK is already set in the cache from a previous build.
set(CATKIN_CURRENT_STACK "" CACHE INTERNAL "" FORCE)
catkin_stack()
catkin_project(catkin)

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
install(FILES
       ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/tests.cmake
       DESTINATION share/catkin/cmake/)

#HACK so that eigen can be found if catkin is in the
#workspace... this is very bad that catkin knows about eigen...
file(COPY cmake/Modules/eigen-config.cmake
  DESTINATION ${CMAKE_BINARY_DIR}/cmake
)

catkin_python_setup()

stamp(cmake/stack_get.py)

install(FILES stack.xml DESTINATION share/catkin)

add_nosetests(${PROJECT_SOURCE_DIR})
