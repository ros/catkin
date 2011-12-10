function(install_cmake_infrastructure PACKAGE_NAME)
  if (NOT PROJECT_NAME STREQUAL PACKAGE_NAME)
    message(FATAL_ERROR "install_cmake_infrastructure called for project (PROJECT_NAME=${PROJECT_NAME}) "
      "that does not match package name argument PACKAGE_NAME=${PACKAGE_NAME}\n"
      "Did you forget to call project()?\n")
  endif()

  parse_arguments(PACKAGE
    "INCLUDE_DIRS;LIBRARIES;CFG_EXTRAS;MSG_DIRS;PYTHONPATH"
    ""
    ${ARGN})

  log(2 "install_cmake_infrastructure ${PACKAGE_NAME} at version ${${PACKAGE_NAME}_VERSION} in @CMAKE_INSTALL_PREFIX@")
  set(pfx ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY})
  set(PACKAGE_NAME ${PACKAGE_NAME})
  if(NOT "${${PACKAGE}_VERSION}" STREQUAL "")
    set(PACKAGE_VERSION ${${PACKAGE}_VERSION})
  else()
    set(PACKAGE_VERSION "0.0.0")
  endif()
  set(PACKAGE_INCLUDE_DIRS ${PACKAGE_INCLUDE_DIRS})
  set(PACKAGE_LIBRARIES ${PACKAGE_LIBRARIES})
  set(PACKAGE_CFG_EXTRAS ${PACKAGE_CFG_EXTRAS})
  set(PACKAGE_CMAKE_CONFIG_FILES_DIR ${CMAKE_INSTALL_PREFIX}/share/cmake/${PACKAGE_NAME})
  set(PACKAGE_MSG_DIRS ${PACKAGE_MSG_DIRS})
  if (PACKAGE_PYTHONPATH)
    set(PACKAGE_PYTHONPATH ${PACKAGE_PYTHONPATH})
    set(${PACKAGE_NAME}_PYTHONPATH ${PACKAGE_PYTHONPATH} CACHE FILEPATH "python path")
  endif()
  # in source
  set(PKG_INCLUDE_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})
  set(PKG_LIB_PREFIX ${CMAKE_CURRENT_BINARY_DIR})
  set(PKG_MSG_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/msg)
  set(PKG_CMAKE_DIR ${CMAKE_BINARY_DIR}/cmake)
  set(PKG_CMAKE_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
  set(PKG_BIN_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/scripts
    ${CMAKE_CURRENT_BINARY_DIR}/bin)
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
  string(TOLOWER ${PACKAGE_NAME} package_lower)
  set(_cfgdir ${CMAKE_BINARY_DIR}/cmake/${package_lower})
  set(_cfgout ${_cfgdir}/${package_lower}-config.cmake)
  log(2 "Writing config to ${_cfgout}")

  file(RELATIVE_PATH PACKAGE_RELATIVE_PATH ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
  if(PACKAGE_RELATIVE_PATH STREQUAL "")
    set(PACKAGE_RELATIVE_PATH ".")
  endif()
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/etc)
  safe_execute_process(COMMAND ${catkin_EXTRAS_DIR}/update_index.py
    ${CMAKE_BINARY_DIR}/etc/packages.list
    "${PACKAGE_NAME}"
    "${PACKAGE_RELATIVE_PATH}"
    )

  #
  #  Install stuff for share/ relative to this.  Maybe we'll want 
  #  to use PACKAGE_RELATIVE_PATH here?
  #
  set(PROJECT_SHARE_INSTALL_PREFIX share/${PACKAGE_NAME})


  # THIS IS IMPORTANT. CMAKE_PREFIX_PATH appears to be twitchy: just
  # spent hours figuring out that having /opt/ros/fuerte first is
  # necessary for this finding to work.  Very strange.  xxx_DIR
  # circumvents this twitchiness.
  set(${PACKAGE_NAME}_DIR ${_cfgdir} CACHE FILEPATH "${PACKAGE_NAME} cmake config file dir")
  configure_file(${catkin_EXTRAS_DIR}/templates/pkg-config.cmake.buildspace.in
    ${_cfgout}
    @ONLY
    )

  configure_file(${catkin_EXTRAS_DIR}/templates/pkg-config-version.cmake.in
    ${_cfgdir}/${package_lower}-config-version.cmake
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

  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/pkg-config)
  em_expand(${catkin_EXTRAS_DIR}/templates/pkg-config.pc.context.in
    ${CMAKE_CURRENT_BINARY_DIR}/pkg-config.pc.context.py
    ${catkin_EXTRAS_DIR}/em/pkg-config.pc.em
    ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PACKAGE_NAME}.pc)

  install(FILES ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PACKAGE_NAME}.pc
    DESTINATION lib/pkgconfig
    )
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

  string(TOLOWER ${PROJECT_NAME} project_lower)
  if(EXISTS ${${PROJECT_NAME}_EXTRAS_DIR}/${project_lower}-config.cmake.in)
    configure_file(${${PROJECT_NAME}_EXTRAS_DIR}/${project_lower}-config.cmake.in
      ${CMAKE_CURRENT_BINARY_DIR}/cmake_install/${project_lower}-config.cmake
      @ONLY
      )
  else()
    configure_file(${catkin_EXTRAS_DIR}/templates/pkg-config.cmake.installable.in
      ${CMAKE_CURRENT_BINARY_DIR}/cmake_install/${project_lower}-config.cmake
      @ONLY
      )
  endif()

  if(EXISTS ${${PROJECT_NAME}_EXTRAS_DIR}/${project_lower}-config-version.cmake.in)
    configure_file(${${PROJECT_NAME}_EXTRAS_DIR}/${project_lower}-config-version.cmake.in
      ${CMAKE_CURRENT_BINARY_DIR}/cmake_install/${project_lower}-config-version.cmake
      @ONLY
      )
  else()
    configure_file(${catkin_EXTRAS_DIR}/templates/pkg-config-version.cmake.in
      ${CMAKE_CURRENT_BINARY_DIR}/cmake_install/${project_lower}-config-version.cmake
      @ONLY
      )
  endif()

  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/cmake_install/${package_lower}-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/cmake_install/${package_lower}-config-version.cmake
    ${INSTALLABLE_CFG_EXTRAS}
    DESTINATION ${PROJECT_SHARE_INSTALL_PREFIX}/cmake
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
	DESTINATION include
        PATTERN .svn EXCLUDE
        )
    else()
      message(WARNING "Include directory '${PACKAGE_INCLUDE_DIRS}' for ${PROJECT_NAME} not found")
    endif()
  endif()

endfunction()
