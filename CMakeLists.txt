cmake_minimum_required(VERSION 2.8)

project(catkin)

# the following lines must be the same as in catkin/cmake/catkin-config.cmake.in
set(catkin_BUILD_PREFIX ${CMAKE_BINARY_DIR}/buildspace CACHE PATH "catkin buildspace")
set(catkin_EXTRAS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake CACHE PATH "catkin extras")
set(catkin_GEN_EXTRAS_DIR ${catkin_BUILD_PREFIX}/share/catkin/cmake CACHE PATH "catkin generated extras")

# generate tests.cmake for buildspace
set(BUILDSPACE TRUE)
set(INSTALLSPACE FALSE)
configure_file(cmake/test/tests.cmake.in
  ${catkin_BUILD_PREFIX}/share/catkin/cmake/tests.cmake
  @ONLY)

# generate and install tests.cmake for installspace
set(BUILDSPACE FALSE)
set(INSTALLSPACE TRUE)
configure_file(cmake/test/tests.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/tests.cmake
  @ONLY)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/tests.cmake
  DESTINATION share/catkin/cmake)

include(${catkin_EXTRAS_DIR}/all.cmake NO_POLICY_SCOPE)

# add additional environment hooks
catkin_add_env_hooks(05.catkin-test-results SHELLS bat sh DIRECTORY ${catkin_EXTRAS_DIR}/env-hooks)

# undefine CATKIN_CURRENT_STACK since it might be set in the cache from a previous build
set(CATKIN_CURRENT_STACK "" CACHE INTERNAL "" FORCE)

catkin_stack()
catkin_project(catkin)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/bin)

# install cmake files for catkin (except some files only used at build time of catkin)
install(DIRECTORY cmake
  DESTINATION share/catkin
  USE_SOURCE_PERMISSIONS
  PATTERN "catkinConfig.cmake.in" EXCLUDE
  PATTERN "test/tests.cmake.in" EXCLUDE)

# XXX HACK so that eigen can be found if catkin is in the
#workspace... this is very bad that catkin knows about eigen...
#file(COPY cmake/Modules/eigen-config.cmake
#  DESTINATION ${CMAKE_BINARY_DIR}/cmake
#)

catkin_python_setup()

# XXX reenable later
#stamp(cmake/stack_get.py)

add_nosetests(${CMAKE_CURRENT_SOURCE_DIR})
