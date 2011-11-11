function(install_cmake_infrastructure PACKAGE_NAME)
  parse_arguments(PACKAGE
    "VERSION;INCLUDE_DIRS;LIBRARIES;CFG_EXTRAS;MSG_DIRS;PYTHONPATH"
    ""
    ${ARGN})

  log(2 "install_cmake_infrastructure ${PACKAGE_NAME} at version ${PACKAGE_VERSION} in @CMAKE_INSTALL_PREFIX@")
  set(pfx ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY})
  set(PACKAGE_NAME ${PACKAGE_NAME})
  set(PACKAGE_VERSION ${PACKAGE_VERSION})
  set(PACKAGE_INCLUDE_DIRS ${PACKAGE_INCLUDE_DIRS})
  set(PACKAGE_LIBRARIES ${PACKAGE_LIBRARIES})
  set(PACKAGE_CFG_EXTRAS ${PACKAGE_CFG_EXTRAS})
  set(PACKAGE_CMAKE_CONFIG_FILES_DIR ${CMAKE_INSTALL_PREFIX}/share/cmake/${PACKAGE_NAME})
  set(PACKAGE_MSG_DIRS ${PACKAGE_MSG_DIRS})
  set(PACKAGE_PYTHONPATH ${PACKAGE_PYTHONPATH})


  # in source
  set(PKG_INCLUDE_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})
  set(PKG_LIB_PREFIX ${CMAKE_CURRENT_BINARY_DIR})
  set(PKG_MSG_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/msg)
  set(PKG_CMAKE_DIR ${CMAKE_BINARY_DIR}/cmake)
  set(PKG_CMAKE_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
  set(PKG_BIN_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/scripts ${CMAKE_CURRENT_BINARY_DIR}/bin)
  set(PKG_LOCATION "Source")

  foreach(extra ${PACKAGE_CFG_EXTRAS})
    assert_file_exists(${CMAKE_CURRENT_SOURCE_DIR}/cmake/${extra}.in "Nonexistent extra")
    configure_file(cmake/${extra}.in
      ${CMAKE_BINARY_DIR}/cmake/${extra}
      @ONLY
      )
  endforeach()

  #
  # Versions find_packageable from the buildspace
  #
  configure_file(${catkin_EXTRAS_DIR}/pkg-config.cmake.in
    ${CMAKE_BINARY_DIR}/cmake/${PACKAGE_NAME}-config.cmake
    @ONLY
    )
  configure_file(${catkin_EXTRAS_DIR}/pkg-config-version.cmake.in
    ${CMAKE_BINARY_DIR}/cmake/${PACKAGE_NAME}-config-version.cmake
    @ONLY
    )

  # installable
  set(PKG_INCLUDE_PREFIX ${CMAKE_INSTALL_PREFIX})
  set(PKG_LIB_PREFIX ${CMAKE_INSTALL_PREFIX})
  set(PKG_MSG_DIRS ${CMAKE_INSTALL_PREFIX}/share/msg/${PACKAGE_NAME})
  set(PKG_CMAKE_DIR ${CMAKE_INSTALL_PREFIX}/share/cmake/${PACKAGE_NAME})
  set(PKG_CMAKE_SRC_DIR ${PKG_CMAKE_DIR})
  set(PKG_BIN_DIRS ${CMAKE_INSTALL_PREFIX}/bin)
  set(PKG_LOCATION "Installed")

  #
  #  Versions to be installed
  #
  set(INSTALLABLE_CFG_EXTRAS "")
  foreach(extra ${PACKAGE_CFG_EXTRAS})
    list(APPEND INSTALLABLE_CFG_EXTRAS ${CMAKE_CURRENT_BINARY_DIR}/cmake_install/${extra})
    configure_file(cmake/${extra}.in
      cmake_install/${extra}
      @ONLY
      )
  endforeach()

  configure_file(${catkin_EXTRAS_DIR}/pkg-config.cmake.in
    cmake_install/${PACKAGE_NAME}-config.cmake
    @ONLY
    )
  configure_file(${catkin_EXTRAS_DIR}/pkg-config-version.cmake.in
    cmake_install/${PACKAGE_NAME}-config-version.cmake
    @ONLY
    )

  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/cmake_install/${PACKAGE_NAME}-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/cmake_install/${PACKAGE_NAME}-config-version.cmake
    ${INSTALLABLE_CFG_EXTRAS}
    DESTINATION share/cmake/${PACKAGE_NAME}
    )

  # install libraries
  if(PACKAGE_LIBRARIES)
    install(TARGETS ${PACKAGE_LIBRARIES}
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib
      )
  endif()

  # install headers, only works for one dir right now
  if(PACKAGE_INCLUDE_DIRS)
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${PACKAGE_INCLUDE_DIRS})
      install(DIRECTORY ${PACKAGE_INCLUDE_DIRS}/
	DESTINATION include)
    else()
      message(WARNING "Include directory '${PACKAGE_INCLUDE_DIRS}' for ${PROJECT_NAME} not found")
    endif()
  endif()

endfunction()
