cmake_minimum_required(VERSION 2.8)

project(catkin)

# the following lines must be the same as in catkin/cmake/catkin-config.cmake.in
set(catkin_BUILD_PREFIX ${CMAKE_BINARY_DIR}/buildspace CACHE PATH "catkin buildspace")
set(catkin_EXTRAS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake CACHE PATH "catkin extras")

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
  PATTERN "catkinConfig.cmake.in" EXCLUDE)

# XXX HACK so that eigen can be found if catkin is in the
#workspace... this is very bad that catkin knows about eigen...
#file(COPY cmake/Modules/eigen-config.cmake
#  DESTINATION ${CMAKE_BINARY_DIR}/cmake
#)

catkin_python_setup()

# XXX reenable later
#stamp(cmake/stack_get.py)

add_nosetests(test)
