function(install_cmake_config_version PACKAGE_NAME_UPPER)
  set(PACKAGE_VERSION ${PACKAGE_VERSION})
  string(TOLOWER ${PACKAGE_NAME_UPPER} PACKAGE_NAME)

  configure_file(${catkin_EXTRAS_DIR}/templates/pkg-config-version.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PACKAGE_NAME}-config-version.cmake
    @ONLY
    )
  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PACKAGE_NAME}-config-version.cmake
    DESTINATION share/cmake/${PACKAGE_NAME_UPPER}
    )
endfunction()