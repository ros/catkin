function(catkin_project catkin_project_name)
  debug_message(10 "catkin_project(${catkin_project_name}) called in file ${CMAKE_CURRENT_LIST_FILE}")

  # verify that catkin_stack() has been called before
  if(NOT CATKIN_CURRENT_STACK)
    message(FATAL_ERROR "catkin_project() CATKIN_CURRENT_STACK is not set.  You must call catkin_stack() in the directory containing stack.xml before you can call catkin_project() in that directory or any of its children.")
  endif()
  # verify that project() has been called before with the same name
  if(NOT catkin_project_name STREQUAL PROJECT_NAME)
    message(FATAL_ERROR "catkin_project() name argument '${catkin_project_name}' does not match current PROJECT_NAME '${PROJECT_NAME}'.  You must call project() with the same name before.")
  endif()

  parse_arguments(PROJECT
    "INCLUDE_DIRS;LIBRARIES;CFG_EXTRAS;DEPENDS"
    ""
    ${ARGN})
  if(PROJECT_DEFAULT_ARGS)
    message(FATAL_ERROR "catkin_project() called with unused arguments: ${PROJECT_DEFAULT_ARGS}")
  endif()

  # set project specific output directory for binaries
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${catkin_BUILD_PREFIX}/lib/${PROJECT_NAME} PARENT_SCOPE)

  # stack version provided by stack.cmake/xml
  set(PROJECT_VERSION ${${CATKIN_CURRENT_STACK}_VERSION})

  # get library paths for all workspaces
  set(lib_paths "")
  foreach(workspace $ENV{CATKIN_WORKSPACES})
    string(REGEX REPLACE ":.*" "" workspace ${workspace})
    list_append_unique(lib_paths ${workspace}/lib)
  endforeach()

  #
  # BUILDSPACE
  #

  set(PKG_BUILDSPACE TRUE)
  set(PKG_INSTALLSPACE FALSE)

  set(PROJECT_SPACE_DIR ${catkin_BUILD_PREFIX})

  # absolute path to include dirs
  set(PROJECT_RELATIVE_INCLUDE_DIRS ${PROJECT_INCLUDE_DIRS})
  # validate that include dirs are existing folders relative to projects source
  set(PROJECT_INCLUDE_DIRS "")
  foreach(relative_include ${PROJECT_RELATIVE_INCLUDE_DIRS})
    set(absolute_include ${CMAKE_CURRENT_SOURCE_DIR}/${relative_include})
    if(NOT EXISTS ${absolute_include})
      message(FATAL_ERROR "catkin_project() include path not found: ${absolute_include}")
    endif()
    list(APPEND PROJECT_INCLUDE_DIRS ${absolute_include})
  endforeach()

  set(PKG_INCLUDE_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})
  # prepend library path of this workspace
  set(PKG_CONFIG_LIB_PATHS ${lib_paths})
  list(INSERT PKG_CONFIG_LIB_PATHS 0 ${PROJECT_SPACE_DIR}/lib)
  set(PKG_CMAKE_DIR ${PROJECT_SPACE_DIR}/share/${PROJECT_NAME}/cmake)

  # ensure that output folder exists
  file(MAKE_DIRECTORY ${catkin_BUILD_PREFIX}/lib/pkgconfig)
  # generate buildspace pc for project
  em_expand(${catkin_EXTRAS_DIR}/templates/pkg.context.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/pkg.buildspace.context.pc.py
    ${catkin_EXTRAS_DIR}/em/pkg.pc.em
    ${catkin_BUILD_PREFIX}/lib/pkgconfig/${PROJECT_NAME}.pc)

  # generate buildspace config for project
  set(infile ${${PROJECT_NAME}_EXTRAS_DIR}/${PROJECT_NAME}Config.cmake.in)
  if(NOT EXISTS ${infile})
    set(infile ${catkin_EXTRAS_DIR}/templates/pkgConfig.cmake.in)
  endif()
  configure_file(${infile}
    ${catkin_BUILD_PREFIX}/share/${PROJECT_NAME}/cmake/${PROJECT_NAME}Config.cmake
    @ONLY
  )

  # generate buildspace config-version for project
  configure_file(${catkin_EXTRAS_DIR}/templates/pkgConfig-version.cmake.in
    ${catkin_BUILD_PREFIX}/share/${PROJECT_NAME}/cmake/${PROJECT_NAME}Config-version.cmake
    @ONLY
  )

  # generate buildspace cfg-extras for project
  foreach(extra ${PROJECT_CFG_EXTRAS})
    assert_file_exists(${CMAKE_CURRENT_SOURCE_DIR}/cmake/${extra}.in "Nonexistent extra")
    #configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/${extra}.in
    configure_file(cmake/${extra}.in
      ${catkin_BUILD_PREFIX}/share/${PROJECT_NAME}/cmake/${extra}
      @ONLY
    )
  endforeach()

  #
  # INSTALLSPACE
  #

  set(PKG_BUILDSPACE FALSE)
  set(PKG_INSTALLSPACE TRUE)

  set(PROJECT_SPACE_DIR ${CMAKE_INSTALL_PREFIX})
  set(PROJECT_INCLUDE_DIRS ${PROJECT_SPACE_DIR}/include)

  set(PKG_INCLUDE_PREFIX "")  # not used in installspace
  # prepend library path of this workspace
  set(PKG_CONFIG_LIB_PATHS ${lib_paths})
  list(INSERT PKG_CONFIG_LIB_PATHS 0 ${PROJECT_SPACE_DIR}/lib)
  set(PKG_CMAKE_DIR ${PROJECT_SPACE_DIR}/share/${PROJECT_NAME}/cmake)

  # ensure that output folder exists
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/installspace)
  # generate and install pc for project
  em_expand(${catkin_EXTRAS_DIR}/templates/pkg.context.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/pkg.installspace.context.pc.py
    ${catkin_EXTRAS_DIR}/em/pkg.pc.em
    ${CMAKE_CURRENT_BINARY_DIR}/installspace/${PROJECT_NAME}.pc)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/installspace/${PROJECT_NAME}.pc
    DESTINATION lib/pkgconfig
  )

  # generate config for project
  set(infile ${${PROJECT_NAME}_EXTRAS_DIR}/${PROJECT_NAME}Config.cmake.in)
  if(NOT EXISTS ${infile})
    set(infile ${catkin_EXTRAS_DIR}/templates/pkgConfig.cmake.in)
  endif()
  configure_file(${infile}
    ${CMAKE_CURRENT_BINARY_DIR}/installspace/${PROJECT_NAME}Config.cmake
    @ONLY
  )

  # generate config-version for project
  set(infile ${${PROJECT_NAME}_EXTRAS_DIR}/${PROJECT_NAME}Config-version.cmake.in)
  if(NOT EXISTS ${infile})
    set(infile ${catkin_EXTRAS_DIR}/templates/pkgConfig-version.cmake.in)
  endif()
  configure_file(${infile}
    ${CMAKE_CURRENT_BINARY_DIR}/installspace/${PROJECT_NAME}Config-version.cmake
    @ONLY
  )

  # generate cfg-extras for project
  set(installable_cfg_extras "")
  foreach(extra ${PROJECT_CFG_EXTRAS})
    list(APPEND installable_cfg_extras ${CMAKE_CURRENT_BINARY_DIR}/installspace/${extra})
    configure_file(cmake/${extra}.in
      ${CMAKE_CURRENT_BINARY_DIR}/installspace/${extra}
      @ONLY
    )
  endforeach()

  # install config, config-version and cfg-extras for project
  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/installspace/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/installspace/${PROJECT_NAME}Config-version.cmake
    ${installable_cfg_extras}
    DESTINATION share/${PROJECT_NAME}/cmake
  )
endfunction()
