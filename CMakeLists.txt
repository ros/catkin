cmake_minimum_required(VERSION 2.8)

project(catkin)

set(INSTALL_SUFFIX share/cmake/catkin)

# in source, do we even want this to be possible for catkin? :)
set(INFRA_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(PKG_LOCATION "Source")

configure_file(cmake/catkin-config.cmake.in
  cmake/catkin-config.cmake
  @ONLY
  )

# installable
set(INFRA_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/${INSTALL_SUFFIX})
set(PKG_LOCATION "Installed")

configure_file(cmake/catkin-config.cmake.in
  cmake_install/catkin-config.cmake
  @ONLY
  )

install(
  FILES
  ${CMAKE_CURRENT_BINARY_DIR}/cmake_install/catkin-config.cmake

  cmake/log.cmake
  cmake/assert.cmake
  cmake/parse_arguments.cmake
  cmake/wg_python.cmake
  cmake/debian-util.cmake
  cmake/empy_cmake.py
  cmake/safe_execute_process.cmake
  cmake/catkin-context.in
  cmake/pkg-config.cmake.in
  cmake/pkg-config-version.cmake.in
  debian.rules
  DESTINATION ${INSTALL_SUFFIX}
  COMPONENT catkin
  )


include(${catkin_BINARY_DIR}/cmake/catkin-config.cmake)
set(catkin_DIR ${catkin_BINARY_DIR}/cmake)
catkin_package(catkin)