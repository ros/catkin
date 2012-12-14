#
# It installs the package.xml file, and it generates code for
# ``find_package`` and ``pkg-config`` so that other packages can get
# information about this package.  For this purpose the information
# about include directories, libraries, further dependencies and
# CMake variables are used.
#
# .. note:: It must be called once for each package.  Best practice
#   is to call this macro early in your root CMakeLists.txt,
#   immediately after calling ``project()`` and
#   ``find_package(catkin REQUIRED)``.
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
# :param CATKIN_DEPENDS: a list of catkin projects which this project
#   depends on.  It is used when client code finds this project via
#   ``find_package()`` or ``pkg-config``.  Each project listed will in
#   turn be ``find_package``\ -ed or is states as ``Requires`` in the
#   .pc file.  Therefore their ``INCLUDE_DIRS`` and ``LIBRARIES`` will
#   be appended to ours.  Only catkin projects should be used where it
#   be guarantee that they are *find_packagable* and have pkg-config
#   files.
# :type CATKIN_DEPENDS: list of strings
# :param DEPENDS: a list of CMake projects which this project depends
#   on.  Since they might not be *find_packagable* or lack a pkg-config
#   file their ``INCLUDE_DIRS`` and ``LIBRARIES`` are passed directly.
#   This requires that it has been ``find_package``\ -ed before.
# :type DEPENDS: list of strings
# :param CFG_EXTRAS: a CMake file containing extra stuff that should
#   be accessible to users of this package after
#   ``find_package``\ -ing it.  This file must live in the
#   subdirectory ``cmake`` and must have the additional extension
#   ``.in`` (since it is expanded using CMake's ``configure_file()``).
#   The template can distinguish between build- and installspace
#   using the boolean variables ``DEVELSPACE`` and ``INSTALLSPACE``
#   and should be verified to work in both cases.
# :type CFG_EXTRAS: string
#
# Example:
# ::
#
#   catkin_package(
#     INCLUDE_DIRS include
#     LIBRARIES projlib1 projlib2
#     CATKIN-DEPENDS roscpp
#     DEPENDS Eigen
#     CFG_EXTRAS proj-extras.cmake
#   )
#
# @public
#
macro(catkin_package)
  debug_message(10 "catkin_package() called in file ${CMAKE_CURRENT_LIST_FILE}")

  # verify that project() has been called before
  if(NOT PROJECT_NAME)
    message(FATAL_ERROR "catkin_package() PROJECT_NAME is not set. You must call project() before calling catkin_package().")
  endif()
  if(PROJECT_NAME STREQUAL "Project")
    message(FATAL_ERROR "catkin_package() PROJECT_NAME is set to 'Project', which is not a valid project name. You must call project() before calling catkin_package().")
  endif()

  # mark that catkin_package() was called in order to detect wrong order of calling with generate_messages()
  set(${PROJECT_NAME}_CATKIN_PACKAGE TRUE)

  # call catkin_package_xml() if it has not been called before
  if(NOT _CATKIN_CURRENT_PACKAGE)
    catkin_package_xml()
  endif()

  _catkin_package(${ARGN})
endmacro()

function(_catkin_package)
  cmake_parse_arguments(PROJECT "" "" "INCLUDE_DIRS;LIBRARIES;CATKIN_DEPENDS;DEPENDS;CFG_EXTRAS" ${ARGN})
  if(PROJECT_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "catkin_package() called with unused arguments: ${PROJECT_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT ${PROJECT_NAME} STREQUAL "catkin")
    list(FIND ${PROJECT_NAME}_BUILDTOOL_DEPENDS "catkin" _index)
    if(_index EQUAL -1)
      list(FIND ${PROJECT_NAME}_BUILD_DEPENDS "catkin" _index)
      if(_index EQUAL -1)
        message(FATAL_ERROR "catkin_package() 'catkin' must be listed as a buildtool dependency in the package.xml")
      endif()
      message("WARNING: 'catkin' should be listed as a buildtool dependency in the package.xml (instead of build dependency)")
    endif()
  endif()

  # prepend INCLUDE_DIRS passed using a variable
  if(${PROJECT_NAME}_INCLUDE_DIRS)
    list(INSERT PROJECT_INCLUDE_DIRS 0 ${${PROJECT_NAME}_INCLUDE_DIRS})
  endif()

  # unset previously found directory of this package, so that this package overlays the other cleanly
  if(${PROJECT_NAME}_DIR)
    set(${PROJECT_NAME}_DIR "" CACHE PATH "" FORCE)
  endif()

  set(_PROJECT_CATKIN_DEPENDS ${PROJECT_CATKIN_DEPENDS})

  set(PROJECT_DEPENDENCIES_INCLUDE_DIRS "")
  set(PROJECT_DEPENDENCIES_LIBRARIES "")
  foreach(depend ${PROJECT_DEPENDS})
    string(REPLACE " " ";" depend_list ${depend})
    # check if the second argument is the COMPONENTS keyword
    list(LENGTH depend_list count)
    set(second_item "")
    if(${count} GREATER 1)
      list(GET depend_list 1 second_item)
    endif()
    if("${second_item}" STREQUAL "COMPONENTS")
      list(GET depend_list 0 depend_name)
      if(NOT ${${depend_name}_FOUND})
        message(FATAL_ERROR "catkin_package() DEPENDS on '${depend}' which must be find_package()-ed before")
      endif()
      message(WARNING "catkin_package() DEPENDS on '${depend}' which is deprecated. find_package() it before and only DEPENDS on '${depend_name}' instead")
      list(APPEND PROJECT_DEPENDENCIES_INCLUDE_DIRS ${${depend_name}_INCLUDE_DIRS})
      list(APPEND PROJECT_DEPENDENCIES_LIBRARIES ${${depend_name}_LIBRARIES})
    else()
      # split multiple names (without COMPONENTS) into separate dependencies
      foreach(depend_name ${depend_list})
        if(${depend_name}_FOUND_CATKIN_PROJECT)
          #message(WARNING "catkin_package() DEPENDS on catkin package '${depend_name}' which is deprecated. Use CATKIN_DEPENDS for catkin packages instead.")
          list(APPEND _PROJECT_CATKIN_DEPENDS ${depend_name})
        else()
          if(NOT ${${depend_name}_FOUND})
            message(FATAL_ERROR "catkin_package() DEPENDS on '${depend_name}' which must be find_package()-ed before. If it is a catkin package it can be declared as CATKIN_DEPENDS instead without find_package()-ing it.")
          endif()
          list(APPEND PROJECT_DEPENDENCIES_INCLUDE_DIRS ${${depend_name}_INCLUDE_DIRS})
          list(APPEND PROJECT_DEPENDENCIES_LIBRARIES ${${depend_name}_LIBRARIES})
        endif()
      endforeach()
    endif()
  endforeach()

  # for catkin packages it can be guaranteed that they are find_package()-able and have pkg-config files
  set(PROJECT_DEPENDENCIES "")
  foreach(depend_name ${_PROJECT_CATKIN_DEPENDS})
    # verify that all catkin packages which have been find_package()-ed are listed as build dependencies
    if(${depend_name}_FOUND)
      # verify that these packages are really catkin packages
      if(NOT ${depend_name}_FOUND_CATKIN_PROJECT)
        message(FATAL_ERROR "catkin_package() CATKIN_DEPENDS on '${depend_name}' which is not a catkin package")
      endif()
      if(catkin_ALL_FOUND_COMPONENTS)
        list(FIND catkin_ALL_FOUND_COMPONENTS ${depend_name} _index)
      else()
        set(_index -1)
      endif()
      if(NOT _index EQUAL -1)
        list(FIND ${PROJECT_NAME}_BUILD_DEPENDS ${depend_name} _index)
        if(_index EQUAL -1)
          message(FATAL_ERROR "catkin_package() the catkin package '${depend_name}' has been find_package()-ed but is not listed as a build dependency in the package.xml")
        endif()
      endif()
    endif()
    # verify that all catkin packages are listed as run dependencies
    list(FIND ${PROJECT_NAME}_RUN_DEPENDS ${depend_name} _index)
    if(_index EQUAL -1)
      message(FATAL_ERROR "catkin_package() DEPENDS on the catkin package '${depend_name}' which must therefore be listed as a run dependency in the package.xml")
    endif()
    list(APPEND PROJECT_DEPENDENCIES ${depend_name})
  endforeach()

  # package version provided by package.cmake/xml
  set(PROJECT_VERSION ${${PROJECT_NAME}_VERSION})

  # flag if package is deprecated provided by package.cmake/xml
  set(PROJECT_DEPRECATED ${${PROJECT_NAME}_DEPRECATED})

  # get library paths from all workspaces
  set(lib_paths "")
  foreach(workspace ${CATKIN_WORKSPACES})
    list_append_unique(lib_paths ${workspace}/lib)
  endforeach()

  # merge explicitly listed libraries and libraries from non-catkin but find_package()-ed packages
  set(_PKG_CONFIG_LIBRARIES "")
  if(PROJECT_LIBRARIES)
    list(APPEND _PKG_CONFIG_LIBRARIES ${PROJECT_LIBRARIES})
  endif()
  if(PROJECT_DEPENDENCIES_LIBRARIES)
    list(APPEND _PKG_CONFIG_LIBRARIES ${PROJECT_DEPENDENCIES_LIBRARIES})
  endif()

  # filter out build configuration keywords and non-matching libraries
  set(PKG_CONFIG_LIBRARIES "")
  list(LENGTH _PKG_CONFIG_LIBRARIES _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _PKG_CONFIG_LIBRARIES ${_index} library)
    if("${library}" STREQUAL "debug")
      if(NOT "${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
        # skip keyword and debug library for non-debug builds
        math(EXPR _index "${_index} + 1")
        if(${_index} EQUAL ${_count})
          message(FATAL_ERROR "catkin_package() the list of libraries '${_PKG_CONFIG_LIBRARIES}' ends with '${library}' which is a build configuration keyword and must be followed by a library")
        endif()
      endif()
    elseif("${library}" STREQUAL "optimized")
      if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
        # skip keyword and non-debug library for debug builds
        math(EXPR _index "${_index} + 1")
        if(${_index} EQUAL ${_count})
          message(FATAL_ERROR "catkin_package() the list of libraries '${_PKG_CONFIG_LIBRARIES}' ends with '${library}' which is a build configuration keyword and must be followed by a library")
        endif()
      endif()
    elseif("${library}" STREQUAL "general")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "catkin_package() the list of libraries '${_PKG_CONFIG_LIBRARIES}' ends with '${library}' which is a build configuration keyword and must be followed by a library")
      endif()
    else()
      list(APPEND PKG_CONFIG_LIBRARIES ${library})
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()

  #
  # DEVEL SPACE
  #

  # used in the cmake extra files
  set(BUILDSPACE TRUE) # this variable is only for backward compatibility
  set(DEVELSPACE TRUE)
  set(INSTALLSPACE FALSE)

  set(PROJECT_SPACE_DIR ${CATKIN_DEVEL_PREFIX})
  set(PKG_INCLUDE_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})

  # absolute path to include dirs and validate that they are existing either absolute or relative to packages source
  set(PROJECT_ABSOLUTE_INCLUDE_DIRS "")
  foreach(idir ${PROJECT_INCLUDE_DIRS})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif(IS_DIRECTORY ${PKG_INCLUDE_PREFIX}/${idir})
      set(include ${PKG_INCLUDE_PREFIX}/${idir})
    else()
      message(FATAL_ERROR "catkin_package() include dir '${idir}' is neither an absolute directory nor exists relative to '${CMAKE_CURRENT_SOURCE_DIR}'")
    endif()
    list(APPEND PROJECT_ABSOLUTE_INCLUDE_DIRS ${include})
  endforeach()
  if(PROJECT_DEPENDENCIES_INCLUDE_DIRS)
    list(APPEND PROJECT_ABSOLUTE_INCLUDE_DIRS ${PROJECT_DEPENDENCIES_INCLUDE_DIRS})
  endif()

  # prepend library path of this workspace
  set(PKG_CONFIG_LIB_PATHS ${lib_paths})
  list(INSERT PKG_CONFIG_LIB_PATHS 0 ${PROJECT_SPACE_DIR}/lib)
  set(PKG_CMAKE_DIR ${PROJECT_SPACE_DIR}/share/${PROJECT_NAME}/cmake)
  if("${PROJECT_NAME}" STREQUAL "catkin")
    set(PKG_CMAKE_DIR "${catkin_EXTRAS_DIR}")
  endif()

  # ensure that output folder exists
  file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/pkgconfig)
  # generate devel space pc for project
  em_expand(${catkin_EXTRAS_DIR}/templates/pkg.context.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/pkg.develspace.context.pc.py
    ${catkin_EXTRAS_DIR}/em/pkg.pc.em
    ${CATKIN_DEVEL_PREFIX}/lib/pkgconfig/${PROJECT_NAME}.pc)

  # generate devel space cfg-extras for project
  set(PKG_CFG_EXTRAS "")
  foreach(extra ${PROJECT_CFG_EXTRAS})
    set(base ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${extra})
    if (EXISTS ${base})
      list(APPEND PKG_CFG_EXTRAS ${base})
    elseif(EXISTS ${base}.em OR EXISTS ${base}.develspace.em)
      if(EXISTS ${base}.em)
        set(em_template ${base}.em)
      else()
        set(em_template ${base}.develspace.em)
      endif()
      em_expand(${catkin_EXTRAS_DIR}/templates/cfg-extras.context.py.in
        ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/${extra}.develspace.context.cmake.py
        ${em_template}
        ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/${extra})
      list(APPEND PKG_CFG_EXTRAS ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/${extra})
    elseif(EXISTS ${base}.in)
      configure_file(${base}.in
        ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/${extra}
        @ONLY
      )
      list(APPEND PKG_CFG_EXTRAS ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/${extra})
    else()
      message(FATAL_ERROR "catkin_package() could not find CFG_EXTRAS file.  Either 'cmake/${extra}', 'cmake/${extra}.em', 'cmake/${extra}.develspace.em' or 'cmake/${extra}.in' must exist.")
    endif()
  endforeach()

  # generate devel space config for project
  set(infile ${${PROJECT_NAME}_EXTRAS_DIR}/${PROJECT_NAME}Config.cmake.in)
  if(NOT EXISTS ${infile})
    set(infile ${catkin_EXTRAS_DIR}/templates/pkgConfig.cmake.in)
  endif()
  configure_file(${infile}
    ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/${PROJECT_NAME}Config.cmake
    @ONLY
  )

  # generate devel space config-version for project
  configure_file(${catkin_EXTRAS_DIR}/templates/pkgConfig-version.cmake.in
    ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/${PROJECT_NAME}Config-version.cmake
    @ONLY
  )

  #
  # INSTALLSPACE
  #

  # used in the cmake extra files
  set(BUILDSPACE FALSE) # this variable is only for backward compatibility
  set(DEVELSPACE FALSE)
  set(INSTALLSPACE TRUE)

  set(PROJECT_SPACE_DIR ${CMAKE_INSTALL_PREFIX})
  set(PKG_INCLUDE_PREFIX ${PROJECT_SPACE_DIR})

  # absolute path to include dir under install prefix if any include dir is set
  set(PROJECT_ABSOLUTE_INCLUDE_DIRS "")
  if(NOT "${PROJECT_INCLUDE_DIRS}" STREQUAL "")
    set(PROJECT_ABSOLUTE_INCLUDE_DIRS ${PKG_INCLUDE_PREFIX}/include)
  endif()
  if(PROJECT_DEPENDENCIES_INCLUDE_DIRS)
    list(APPEND PROJECT_ABSOLUTE_INCLUDE_DIRS ${PROJECT_DEPENDENCIES_INCLUDE_DIRS})
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

  # generate and install cfg-extras for project
  set(PKG_CFG_EXTRAS "")
  set(installable_cfg_extras "")
  foreach(extra ${PROJECT_CFG_EXTRAS})
    set(base ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${extra})
    if (EXISTS ${base})
      list(APPEND installable_cfg_extras ${base})
    elseif(EXISTS ${base}.em OR EXISTS ${base}.installspace.em)
      if(EXISTS ${base}.em)
        set(em_template ${base}.em)
      else()
        set(em_template ${base}.installspace.em)
      endif()
      em_expand(${catkin_EXTRAS_DIR}/templates/cfg-extras.context.py.in
        ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/${extra}.installspace.context.cmake.py
        ${em_template}
        ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${extra})
      list(APPEND installable_cfg_extras ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${extra})
    elseif(EXISTS ${base}.in)
      configure_file(${base}.in
        ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${extra}
        @ONLY
      )
      list(APPEND installable_cfg_extras ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${extra})
    else()
      message(FATAL_ERROR "catkin_package() could not find CFG_EXTRAS file.  Either 'cmake/${extra}', 'cmake/${extra}.em', 'cmake/${extra}.installspace.em' or 'cmake/${extra}.in' must exist.")
    endif()
    list(APPEND PKG_CFG_EXTRAS ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/cmake/${extra})
  endforeach()
  install(FILES
    ${installable_cfg_extras}
    DESTINATION share/${PROJECT_NAME}/cmake
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

  # install config, config-version and cfg-extras for project
  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${PROJECT_NAME}Config-version.cmake
    DESTINATION share/${PROJECT_NAME}/cmake
  )

  # install package.xml
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
    DESTINATION share/${PROJECT_NAME}
  )
endfunction()
