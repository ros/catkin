cmake_minimum_required(VERSION 2.8)

project(catkin)

# the following lines must be the same as in catkin/cmake/catkinConfig.cmake.in
set(CATKIN_BUILD_PREFIX ${CMAKE_BINARY_DIR}/buildspace CACHE PATH "catkin buildspace")
set(catkin_EXTRAS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake CACHE PATH "catkin extras")
list(INSERT CMAKE_PREFIX_PATH 0 ${CMAKE_BINARY_DIR}/buildspace)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} PARENT_SCOPE)

include(${catkin_EXTRAS_DIR}/all.cmake NO_POLICY_SCOPE)

# undefine CATKIN_CURRENT_STACK since it might be set in the cache from a previous build
set(CATKIN_CURRENT_STACK "" CACHE INTERNAL "" FORCE)

catkin_stack()
catkin_project(catkin)

# for backward compatibility we provide ROSConfig(-version).cmake files
file(COPY cmake/ROSConfig.cmake
  DESTINATION ${CATKIN_BUILD_PREFIX}/share/ros/cmake
)
set(PROJECT_VERSION ${catkin_VERSION})
configure_file(${catkin_EXTRAS_DIR}/templates/pkgConfig-version.cmake.in
  ${CATKIN_BUILD_PREFIX}/share/ros/cmake/ROSConfig-version.cmake
  @ONLY
)
install(FILES ${CATKIN_BUILD_PREFIX}/share/ros/cmake/ROSConfig.cmake ${CATKIN_BUILD_PREFIX}/share/ros/cmake/ROSConfig-version.cmake
  DESTINATION share/ros/cmake
)

# for compatibility we provide eigen-config(-version).cmake files
file(COPY cmake/Modules/eigen-config.cmake cmake/Modules/eigen-config-version.cmake
  DESTINATION ${CATKIN_BUILD_PREFIX}/share/eigen/cmake
)
install(FILES cmake/Modules/eigen-config.cmake cmake/Modules/eigen-config-version.cmake
  DESTINATION share/eigen/cmake
)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/bin)

# install cmake files for catkin (except some files only used at build time of catkin)
install(DIRECTORY cmake
  DESTINATION share/catkin
  USE_SOURCE_PERMISSIONS
  PATTERN "catkinConfig.cmake.in" EXCLUDE)

catkin_python_setup()

catkin_add_nosetests(test)
