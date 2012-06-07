function(catkin_project PACKAGE_NAME)
  # catkin_stack() is required first, #78
  if(NOT CATKIN_CURRENT_STACK)
    message(FATAL_ERROR "catkin_project(): CATKIN_CURRENT_STACK is unset.  You must call catkin_stack() in the directory containing stack.xml before you can call catkin_project() in that directory or any of its children.")
  endif()
  if (NOT PROJECT_NAME STREQUAL PACKAGE_NAME)
    message(FATAL_ERROR "catkin_project called for project (PROJECT_NAME=${PROJECT_NAME}) "
      "that does not match package name argument PACKAGE_NAME=${PACKAGE_NAME}\n"
      "Did you forget to call project()?\n")
  endif()
  set(Maintainer ${${CATKIN_CURRENT_STACK}_MAINTAINER})

  parse_arguments(PACKAGE
    "INCLUDE_DIRS;LIBRARIES;CFG_EXTRAS;PYTHONPATH;DEPENDS"
    ""
    ${ARGN})

  check_unused_arguments("catkin_project" "${PACKAGE_DEFAULT_ARGS}")

  if (PACKAGE_PYTHONPATH)
    message(WARNING "PYTHONPATH in catkin_project is deprecated. Please remove for pkg: ${PACKAGE_NAME}")
  endif()

  log(2 "catkin_project ${PACKAGE_NAME} at version ${${PACKAGE_NAME}_VERSION} in @CMAKE_INSTALL_PREFIX@")
  set(pfx ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY})
  set(PACKAGE_NAME ${PACKAGE_NAME})
  # ${${CATKIN_CURRENT_STACK}_VERSION} is set by the most recent call to
  # catkin_stack(), which sets CATKIN_CURRENT_STACK and then parses the
  # stack.xml, making each field into a CMake cache variable.
  set(PACKAGE_VERSION ${${CATKIN_CURRENT_STACK}_VERSION})
  set(PACKAGE_INCLUDE_DIRS ${PACKAGE_INCLUDE_DIRS})
  set(PACKAGE_DEPENDS ${PACKAGE_DEPENDS})
  set(PACKAGE_LIBRARIES ${PACKAGE_LIBRARIES})
  set(PACKAGE_CFG_EXTRAS ${PACKAGE_CFG_EXTRAS})
  set(PACKAGE_CMAKE_CONFIG_FILES_DIR ${CMAKE_INSTALL_PREFIX}/share/${PACKAGE_NAME}/cmake)

  #
  # Default executable output location to "private".
  #
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin PARENT_SCOPE)

  # force new policy for escaping preprocessor definitions
  cmake_policy(SET CMP0005 NEW)
  # Add ROS_PACKAGE_NAME define
  add_definitions(-DROS_PACKAGE_NAME=\"${PROJECT_NAME}\")

  #
  # Versions find_packageable from the buildspace
  #
  string(TOLOWER ${PACKAGE_NAME} package_lower)
  set(_cfgdir ${CMAKE_BINARY_DIR}/cmake/${package_lower})
  set(_cfgout ${_cfgdir}/${package_lower}-config.cmake)
  log(2 "Writing config to ${_cfgout}")

  # in source
  set(PKG_INCLUDE_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})
  set(PKG_LIB_PREFIX ${CMAKE_CURRENT_BINARY_DIR})
  set(PKG_CMAKE_DIR ${_cfgdir})
  set(PKG_CMAKE_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
  set(PKG_BIN_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/scripts
    ${CMAKE_CURRENT_BINARY_DIR}/bin)
  set(PKG_LOCATION "Source")

  #put all extras into the cfg dir...
  foreach(extra ${PACKAGE_CFG_EXTRAS})
    assert_file_exists(${CMAKE_CURRENT_SOURCE_DIR}/cmake/${extra}.in "Nonexistent extra")
    configure_file(cmake/${extra}.in
      ${_cfgdir}/${extra}
      @ONLY
      )
  endforeach()

  file(RELATIVE_PATH PACKAGE_RELATIVE_PATH ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
  if(PACKAGE_RELATIVE_PATH STREQUAL "")
    set(PACKAGE_RELATIVE_PATH ".")
  endif()
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/etc)
  safe_execute_process(COMMAND ${PYTHON_EXECUTABLE} ${catkin_EXTRAS_DIR}/update_index.py
    ${CMAKE_BINARY_DIR}/etc/packages.list
    "${PACKAGE_NAME}"
    "${PACKAGE_RELATIVE_PATH}"
    )

  #
  #  Install stuff for share/ relative to this.  Maybe we'll want
  #  to use PACKAGE_RELATIVE_PATH here?
  #
  set(PROJECT_SHARE_INSTALL_PREFIX share/${PACKAGE_NAME})


  #EAR: Why can't the buildspace config files be overridden from the ${PROJECT_NAME}_EXTRAS_DIR ?

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
  set(PKG_CMAKE_DIR ${CMAKE_INSTALL_PREFIX}/share/${PACKAGE_NAME}/cmake)
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

  string(TOLOWER ${PROJECT_NAME} project_lower)

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
endfunction()
