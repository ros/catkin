cmake_minimum_required(VERSION 2.8)

project(rosbuild)

set(INSTALL_SUFFIX share/cmake/${PROJECT_NAME})

# in source, do we even want this to be possible for rosbuild? :)
set(INFRA_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(PKG_LOCATION "Source")

configure_file(cmake/${PROJECT_NAME}-config.cmake.in
  cmake/${PROJECT_NAME}-config.cmake
  @ONLY
  )

# installable
set(INFRA_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/${INSTALL_SUFFIX})
set(PKG_LOCATION "Installed")

configure_file(cmake/${PROJECT_NAME}-config.cmake.in
  cmake_install/${PROJECT_NAME}-config.cmake
  @ONLY
  )

install(
  FILES
  ${CMAKE_CURRENT_BINARY_DIR}/cmake_install/${PROJECT_NAME}-config.cmake

  cmake/log.cmake
  cmake/assert.cmake
  cmake/parse_arguments.cmake
  cmake/wg_python.cmake
  cmake/debian-util.cmake
  cmake/safe_execute_process.cmake

  cmake/pkg-config.cmake.in
  cmake/pkg-config-version.cmake.in
  DESTINATION ${INSTALL_SUFFIX}
  )
