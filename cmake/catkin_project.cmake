#
# It creates the CMake stuff necessary for ``find_package`` to work
# (i.e. to be *found* by others that call ``find_package``) and
# provide information about include directories, libraries,
# further dependencies and CMake variables for dependent projects.
#
# Best practice is to call this macro early in your CMakeLists.txt,
# immediately after calling ``project()`` and ``find_package(catkin REQUIRED)``.
#
# :param catkin_project_name: requires the same value as passed to
#   CMake's ``project()`` (*may be vestigial*)
# :param INCLUDE_DIRS: ``CMAKE_CURRENT_SOURCE_DIR``-relative paths to
#   C/C++ includes
# :param LIBRARIES: names of library targets that will appear in the
#   ``catkin_LIBRARIES`` and ``${PROJECT_NAME}_LIBRARIES of other
#   projects that search for you via ``find_package``.  Currently
#   this will break if the logical target names are not the same as
#   the installed names.
# :param DEPENDS: The argument ``DEPENDS`` is used when client code
#   finds us via ``find_package()``.  Each project listed will in
#   turn be ``find_package``\ -ed and their ``INCLUDE_DIRS`` and
#   ``LIBRARIES`` will be appended to ours.  Only catkin projects
#   should be used where we can ensure that they are
#   *find_packagable* and *package_configurable*.
# :param CFG_EXTRAS: Any extra CMake stuff that should be accessible
#   to users of the project.  This file should live in subdirectory
#   ``cmake`` and have extension ``.in``.  It will be expanded by
#   CMake's ``configure_file()`` and made available to clients in
#   both the install and build spaces: be sure it works both ways.
#   TODO: document which variables are available in these templates
#   (boolean @PKG_BUILDSPACE@, boolean @PKG_INSTALLSPACE@).
# :outvar PROJECT_INCLUDE_DESTINATION: set to
#   ``include``.
#   For use with CMake ``install()`` macro as destination argument.
# :outvar PROJECT_LIB_DESTINATION: set to
#   ``lib``.
#   For use with CMake ``install()`` macro as destination argument.
# :outvar PROJECT_LIBEXEC_DESTINATION: set to
#   ``lib/${PROJECT_NAME}``.
#   For use with CMake ``install()`` macro as destination argument.
# :outvar PROJECT_PYTHON_DESTINATION: set to
#   ``lib/python${PYTHON_VERSION_XDOTY}/${PYTHON_PACKAGES_DIR}/${PROJECT_NAME}``.
#   For use with CMake ``install()`` macro as destination argument.
# :outvar PROJECT_SHARE_DESTINATION: set to
#   ``share/${PROJECT_NAME}``.
#   For use with CMake ``install()`` macro as destination argument.
#
# Example:
# ::
#   catkin_project(proj
#     INCLUDE_DIRS include
#     LIBRARIES proj-one proj-two
#     DEPENDS roscpp
#     CFG_EXTRAS proj-extras.cmake
#   )
#
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
  foreach(workspace ${CATKIN_WORKSPACES})
    list_append_unique(lib_paths ${workspace}/lib)
  endforeach()

  #
  # BUILDSPACE
  #

  set(PKG_BUILDSPACE TRUE)
  set(PKG_INSTALLSPACE FALSE)

  set(PROJECT_SPACE_DIR ${catkin_BUILD_PREFIX})
  set(PKG_INCLUDE_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})

  # absolute path to include dirs and validate that they are existing either absolute or relative to projects source
  set(PROJECT_ABSOLUTE_INCLUDE_DIRS "")
  foreach(idir ${PROJECT_INCLUDE_DIRS})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif(IS_DIRECTORY ${PKG_INCLUDE_PREFIX}/${idir})
      set(include ${PKG_INCLUDE_PREFIX}/${idir})
    else()
      message(FATAL_ERROR "catkin_project() include dir '${idir}' is neither an absolute directory nor exists relative to '${CMAKE_CURRENT_SOURCE_DIR}'")
    endif()
    list(APPEND PROJECT_ABSOLUTE_INCLUDE_DIRS ${include})
  endforeach()

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
  set(PKG_INCLUDE_PREFIX ${PROJECT_SPACE_DIR})

  # absolute path to include dir under install prefix if any include dir is set
  set(PROJECT_ABSOLUTE_INCLUDE_DIRS "")
  if(${PROJECT_INCLUDE_DIRS})
    set(PROJECT_ABSOLUTE_INCLUDE_DIRS ${PKG_INCLUDE_PREFIX}/include)
  endif()

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
