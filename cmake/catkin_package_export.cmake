#
# Set a name for the catkin package.
#
# It creates the CMake stuff necessary for ``find_package`` to work
# (i.e. to be *found* by others that call ``find_package``) and
# provide information about include directories, libraries,
# further dependencies and CMake variables for dependent packages.
#
# .. note:: It must be called once for each package.  Best practice
#   is to call this macro early in your root CMakeLists.txt,
#   immediately after calling ``project()``,
#   ``find_package(catkin REQUIRED)`` and ``catkin_package()``.
#
# :param INCLUDE_DIRS: ``CMAKE_CURRENT_SOURCE_DIR``-relative paths to
#   C/C++ includes
# :type INCLUDE_DIRS: list of strings
# :param LIBRARIES: names of library targets that will appear in the
#   ``catkin_LIBRARIES`` and ``${PROJECT_NAME}_LIBRARIES`` of other
#   projects that search for you via ``find_package``.  Currently
#   this will break if the logical target names are not the same as
#   the installed names.
# :type LIBRARIES: list of strings
# :param DEPENDS: a list of CMake projects which this project depends
#   on.  It is used when client code finds this project via
#   ``find_package()``.  Each project listed will in turn be
#   ``find_package``\ -ed and their ``INCLUDE_DIRS`` and ``LIBRARIES``
#   will be appended to ours.  Only projects should be used where we
#   can guarantee that they are *find_packagable*. If they are not
#   catkin packages they are not added to the ``requires`` list in
#   the pkg-config file since we can not ensure that they are
#   *package_configurable*.
# :type DEPENDS: list of strings
# :param CFG_EXTRAS: a CMake file containing extra stuff that should
#   be accessible to users of this package after
#   ``find_package``\ -ing it.  This file must live in the
#   subdirectory ``cmake`` and must have the additional extension
#   ``.in`` (since it is expanded using CMake's ``configure_file()``).
#   The template can distinguish between build- and installspace
#   using the boolean variables ``BUILDSPACE`` and ``INSTALLSPACE``
#   and should be verified to work in both cases.
# :type CFG_EXTRAS: string
#
# :outvar CATKIN_PACKAGE_BIN_DESTINATION:
#   See :cmake:data:`CATKIN_PACKAGE_BIN_DESTINATION`.
# :outvar CATKIN_PACKAGE_ETC_DESTINATION:
#   See :cmake:data:`CATKIN_PACKAGE_ETC_DESTINATION`.
# :outvar CATKIN_PACKAGE_INCLUDE_DESTINATION:
#   See :cmake:data:`CATKIN_PACKAGE_INCLUDE_DESTINATION`.
# :outvar CATKIN_PACKAGE_LIB_DESTINATION:
#   See :cmake:data:`CATKIN_PACKAGE_LIB_DESTINATION`.
# :outvar CATKIN_PACKAGE_PYTHON_DESTINATION:
#   See :cmake:data:`CATKIN_PACKAGE_PYTHON_DESTINATION`.
# :outvar CATKIN_PACKAGE_SHARE_DESTINATION:
#   See :cmake:data:`CATKIN_PACKAGE_SHARE_DESTINATION`.
#
# :outvar CATKIN_GLOBAL_BIN_DESTINATION:
#   See :cmake:data:`CATKIN_GLOBAL_BIN_DESTINATION`.
# :outvar CATKIN_GLOBAL_ETC_DESTINATION:
#   See :cmake:data:`CATKIN_GLOBAL_ETC_DESTINATION`.
# :outvar CATKIN_GLOBAL_INCLUDE_DESTINATION:
#   See :cmake:data:`CATKIN_GLOBAL_INCLUDE_DESTINATION`.
# :outvar CATKIN_GLOBAL_LIB_DESTINATION:
#   See :cmake:data:`CATKIN_GLOBAL_LIB_DESTINATION`.
# :outvar CATKIN_GLOBAL_LIBEXEC_DESTINATION:
#   See :cmake:data:`CATKIN_GLOBAL_LIBEXEC_DESTINATION`.
# :outvar CATKIN_GLOBAL_PYTHON_DESTINATION:
#   See :cmake:data:`CATKIN_GLOBAL_PYTHON_DESTINATION`.
# :outvar CATKIN_GLOBAL_SHARE_DESTINATION:
#   See :cmake:data:`CATKIN_GLOBAL_SHARE_DESTINATION`.
#
# Example:
# ::
#
#   catkin_package_export(
#     INCLUDE_DIRS include
#     LIBRARIES proj-one proj-two
#     DEPENDS roscpp
#     CFG_EXTRAS proj-extras.cmake
#   )
#
# @public
#
function(catkin_package_export)
  debug_message(10 "catkin_package_export() called in file ${CMAKE_CURRENT_LIST_FILE}")

  # verify that catkin_package() has been called before
  if(NOT _CATKIN_CURRENT_PACKAGE)
    message(FATAL_ERROR "catkin_package_export() _CATKIN_CURRENT_PACKAGE is not set.  You must call catkin_package() in the directory containing package.xml before you can call catkin_package_export().")
  endif()

  parse_arguments(PROJECT
    "INCLUDE_DIRS;LIBRARIES;CFG_EXTRAS;DEPENDS"
    ""
    ${ARGN})
  if(PROJECT_DEFAULT_ARGS)
    message(FATAL_ERROR "catkin_package_export() called with unused arguments: ${PROJECT_DEFAULT_ARGS}")
  endif()

  # unset previously found directory of this package, so that this package overlays the other cleanly
  if(${PROJECT_NAME}_DIR)
    set(${PROJECT_NAME}_DIR "" CACHE PATH "" FORCE)
  endif()

  # filter out DEPENDS which have not been find_package()-ed before
  foreach(depend ${PROJECT_DEPENDS})
    if(NOT ${depend}_FOUND)
      message(WARNING "catkin_package_export(${PROJECT_NAME}) depends on '${depend}' which has not been find_package()-ed before")
      list(REMOVE_ITEM PROJECT_DEPENDS ${depend})
    endif()
  endforeach()

  # find catkin-only packages in DEPENDS list for use in .pc files
  set(PROJECT_CATKIN_DEPENDS "")
  foreach(depend ${PROJECT_DEPENDS})
    if(${${depend}_FOUND_CATKIN_PROJECT})
      list(APPEND PROJECT_CATKIN_DEPENDS ${depend})
    endif()
  endforeach()

  # package version provided by package.cmake/xml
  set(PROJECT_VERSION ${${PROJECT_NAME}_VERSION})

  # get library paths from all workspaces
  set(lib_paths "")
  foreach(workspace ${CATKIN_WORKSPACES})
    list_append_unique(lib_paths ${workspace}/lib)
  endforeach()

  #
  # BUILDSPACE
  #

  set(BUILDSPACE TRUE)
  set(INSTALLSPACE FALSE)

  set(PROJECT_SPACE_DIR ${CATKIN_BUILD_PREFIX})
  set(PKG_INCLUDE_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})

  # absolute path to include dirs and validate that they are existing either absolute or relative to packages source
  set(PROJECT_ABSOLUTE_INCLUDE_DIRS "")
  foreach(idir ${PROJECT_INCLUDE_DIRS})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif(IS_DIRECTORY ${PKG_INCLUDE_PREFIX}/${idir})
      set(include ${PKG_INCLUDE_PREFIX}/${idir})
    else()
      message(FATAL_ERROR "catkin_package_export() include dir '${idir}' is neither an absolute directory nor exists relative to '${CMAKE_CURRENT_SOURCE_DIR}'")
    endif()
    list(APPEND PROJECT_ABSOLUTE_INCLUDE_DIRS ${include})
  endforeach()

  # prepend library path of this workspace
  set(PKG_CONFIG_LIB_PATHS ${lib_paths})
  list(INSERT PKG_CONFIG_LIB_PATHS 0 ${PROJECT_SPACE_DIR}/lib)
  set(PKG_CMAKE_DIR ${PROJECT_SPACE_DIR}/share/${PROJECT_NAME}/cmake)

  # ensure that output folder exists
  file(MAKE_DIRECTORY ${CATKIN_BUILD_PREFIX}/lib/pkgconfig)
  # generate buildspace pc for project
  em_expand(${catkin_EXTRAS_DIR}/templates/pkg.context.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/pkg.buildspace.context.pc.py
    ${catkin_EXTRAS_DIR}/em/pkg.pc.em
    ${CATKIN_BUILD_PREFIX}/lib/pkgconfig/${PROJECT_NAME}.pc)

  # generate buildspace config for project
  set(infile ${${PROJECT_NAME}_EXTRAS_DIR}/${PROJECT_NAME}Config.cmake.in)
  if(NOT EXISTS ${infile})
    set(infile ${catkin_EXTRAS_DIR}/templates/pkgConfig.cmake.in)
  endif()
  configure_file(${infile}
    ${CATKIN_BUILD_PREFIX}/share/${PROJECT_NAME}/cmake/${PROJECT_NAME}Config.cmake
    @ONLY
  )

  # generate buildspace config-version for project
  configure_file(${catkin_EXTRAS_DIR}/templates/pkgConfig-version.cmake.in
    ${CATKIN_BUILD_PREFIX}/share/${PROJECT_NAME}/cmake/${PROJECT_NAME}Config-version.cmake
    @ONLY
  )

  # generate buildspace cfg-extras for project
  foreach(extra ${PROJECT_CFG_EXTRAS})
    assert_file_exists(${CMAKE_CURRENT_SOURCE_DIR}/cmake/${extra}.in "Nonexistent extra")
    #configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/${extra}.in
    configure_file(cmake/${extra}.in
      ${CATKIN_BUILD_PREFIX}/share/${PROJECT_NAME}/cmake/${extra}
      @ONLY
    )
  endforeach()

  #
  # INSTALLSPACE
  #

  set(BUILDSPACE FALSE)
  set(INSTALLSPACE TRUE)

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
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace)
  # generate and install pc for project
  em_expand(${catkin_EXTRAS_DIR}/templates/pkg.context.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/pkg.installspace.context.pc.py
    ${catkin_EXTRAS_DIR}/em/pkg.pc.em
    ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${PROJECT_NAME}.pc)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${PROJECT_NAME}.pc
    DESTINATION lib/pkgconfig
  )

  # generate config for project
  set(infile ${${PROJECT_NAME}_EXTRAS_DIR}/${PROJECT_NAME}Config.cmake.in)
  if(NOT EXISTS ${infile})
    set(infile ${catkin_EXTRAS_DIR}/templates/pkgConfig.cmake.in)
  endif()
  configure_file(${infile}
    ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${PROJECT_NAME}Config.cmake
    @ONLY
  )

  # generate config-version for project
  set(infile ${${PROJECT_NAME}_EXTRAS_DIR}/${PROJECT_NAME}Config-version.cmake.in)
  if(NOT EXISTS ${infile})
    set(infile ${catkin_EXTRAS_DIR}/templates/pkgConfig-version.cmake.in)
  endif()
  configure_file(${infile}
    ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${PROJECT_NAME}Config-version.cmake
    @ONLY
  )

  # generate cfg-extras for project
  set(installable_cfg_extras "")
  foreach(extra ${PROJECT_CFG_EXTRAS})
    list(APPEND installable_cfg_extras ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${extra})
    configure_file(cmake/${extra}.in
      ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${extra}
      @ONLY
    )
  endforeach()

  # install config, config-version and cfg-extras for project
  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${PROJECT_NAME}Config-version.cmake
    ${installable_cfg_extras}
    DESTINATION share/${PROJECT_NAME}/cmake
  )

  #
  # BUILD AND INSTALL DESTINATIONS
  #

  # set project specific output directory for libraries
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CATKIN_BUILD_PREFIX}/lib PARENT_SCOPE)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CATKIN_BUILD_PREFIX}/lib PARENT_SCOPE)
  # set project specific output directory for binaries
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CATKIN_BUILD_PREFIX}/lib/${PROJECT_NAME} PARENT_SCOPE)

  # set global install destinations
  set(CATKIN_GLOBAL_BIN_DESTINATION bin PARENT_SCOPE)
  set(CATKIN_GLOBAL_ETC_DESTINATION etc PARENT_SCOPE)
  set(CATKIN_GLOBAL_INCLUDE_DESTINATION include PARENT_SCOPE)
  set(CATKIN_GLOBAL_LIB_DESTINATION lib PARENT_SCOPE)
  set(CATKIN_GLOBAL_LIBEXEC_DESTINATION lib PARENT_SCOPE)
  set(CATKIN_GLOBAL_PYTHON_DESTINATION ${PYTHON_INSTALL_DIR} PARENT_SCOPE)
  set(CATKIN_GLOBAL_SHARE_DESTINATION share PARENT_SCOPE)

  # set project specific install destinations
  set(CATKIN_PROJECT_BIN_DESTINATION lib/${PROJECT_NAME} PARENT_SCOPE)
  set(CATKIN_PROJECT_ETC_DESTINATION etc/${PROJECT_NAME} PARENT_SCOPE)
  set(CATKIN_PROJECT_INCLUDE_DESTINATION include/${PROJECT_NAME} PARENT_SCOPE)
  set(CATKIN_PROJECT_LIB_DESTINATION lib PARENT_SCOPE)
  set(CATKIN_PROJECT_PYTHON_DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME} PARENT_SCOPE)
  set(CATKIN_PROJECT_SHARE_DESTINATION share/${PROJECT_NAME} PARENT_SCOPE)

  # XXXX generate manifest.xml for backward compatibility
endfunction()
