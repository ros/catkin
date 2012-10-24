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
#   catkin_package(
#     INCLUDE_DIRS include
#     LIBRARIES proj-one proj-two
#     DEPENDS roscpp
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

  # call catkin_package_xml() if it has not been called manually before
  if(NOT _CATKIN_CURRENT_PACKAGE)
    catkin_package_xml()
  endif()

  #
  # BUILD AND INSTALL DESTINATIONS
  #

  # set project specific install destinations
  set(CATKIN_PACKAGE_BIN_DESTINATION ${CATKIN_GLOBAL_LIBEXEC_DESTINATION}/${PROJECT_NAME})
  set(CATKIN_PACKAGE_ETC_DESTINATION ${CATKIN_GLOBAL_ETC_DESTINATION}/${PROJECT_NAME})
  set(CATKIN_PACKAGE_INCLUDE_DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME})
  set(CATKIN_PACKAGE_LIB_DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION})
  set(CATKIN_PACKAGE_PYTHON_DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}/${PROJECT_NAME})
  set(CATKIN_PACKAGE_SHARE_DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/${PROJECT_NAME})

  # set project specific output directory for libraries
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CATKIN_BUILD_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CATKIN_BUILD_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
  # set project specific output directory for binaries
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CATKIN_BUILD_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})

  _catkin_package(${ARGN})
endmacro()

function(_catkin_package)
  _parse_arguments_with_repeated_keywords(PROJECT "" "" "INCLUDE_DIRS;LIBRARIES;CFG_EXTRAS" "DEPENDS" ${ARGN})
  if(PROJECT_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "catkin_package() called with unused arguments: ${PROJECT_UNPARSED_ARGUMENTS}")
  endif()

  # unset previously found directory of this package, so that this package overlays the other cleanly
  if(${PROJECT_NAME}_DIR)
    set(${PROJECT_NAME}_DIR "" CACHE PATH "" FORCE)
  endif()

  # split DEPENDS with multiple packages into separate items
  set(PROJECT_DEPENDENCIES "")
  foreach(depend ${PROJECT_DEPENDS})
    string(REPLACE " " ";" depend_list ${depend})
    # check if the second argument is the COMPONENTS keyword
    list(LENGTH depend_list count)
    set(second_item "")
    if(${count} GREATER 1)
      list(GET depend_list 1 second_item)
    endif()
    if("${second_item}" STREQUAL "COMPONENTS")
      # pass dependencies with components as is
      list(APPEND PROJECT_DEPENDENCIES ${depend})
    else()
      # split multiple names (without components) into separate dependencies
      foreach(dep ${depend_list})
        list(APPEND PROJECT_DEPENDENCIES ${dep})
      endforeach()
    endif()
  endforeach()

  # filter out dependencies which have not been find_package()-ed before
  foreach(depend ${PROJECT_DEPENDENCIES})
    string(REPLACE " " ";" depend_list ${depend})
    list(GET depend_list 0 depend_name)
    if(NOT ${depend_name}_FOUND)
      message(WARNING "catkin_package(${PROJECT_NAME}) depends on '${depend_name}' which has not been find_package()-ed before")
      list(REMOVE_ITEM PROJECT_DEPENDENCIES ${depend})
    endif()
  endforeach()

  # extract all catkin packages from dependencies
  set(PROJECT_CATKIN_DEPENDS "")
  foreach(depend ${PROJECT_DEPENDENCIES})
    string(REPLACE " " ";" depend_list ${depend})
    # check if dependency is a catkin package
    list(GET depend_list 0 depend_name)
    if(${${depend_name}_FOUND_CATKIN_PROJECT})
      list(APPEND PROJECT_CATKIN_DEPENDS ${depend_name})
      # verify that all catkin dependencies are listed as build- and runtime dependencies
      list(FIND ${PROJECT_NAME}_BUILD_DEPENDS ${depend_name} _index)
      if(_index EQUAL -1)
        message(FATAL_ERROR "catkin_package(${PROJECT_NAME}) depends on catkin package '${depend_name}' which must therefore be listed as a build dependency in the package.xml")
      endif()
      list(FIND ${PROJECT_NAME}_RUN_DEPENDS ${depend_name} _index)
      if(_index EQUAL -1)
        message(FATAL_ERROR "catkin_package(${PROJECT_NAME}) depends on catkin package '${depend_name}' which must therefore be listed as a run dependency in the package.xml")
      endif()
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
      message(FATAL_ERROR "catkin_package include dir '${idir}' is neither an absolute directory nor exists relative to '${CMAKE_CURRENT_SOURCE_DIR}'")
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
  if(NOT "X${PROJECT_INCLUDE_DIRS}" STREQUAL "X")
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

  # install package.xml
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
    DESTINATION share/${PROJECT_NAME}
  )
endfunction()


# The following function is derived from CMake's cmake_parse_arguments function.
# The support of repeated keywords has been added for catkin_package().
# For each keyword a list variable is returned where each item contains all arguments separated by a whitespace.
# The original license of the file is: 
#=============================================================================
# Copyright 2010 Alexander Neundorf <neundorf@kde.org>
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# Full CMake Copyright notice
#
# Copyright 2000-2009 Kitware, Inc., Insight Software Consortium. All rights
# reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# Neither the names of Kitware, Inc., the Insight Software Consortium, nor
# the names of their contributors may be used to endorse or promote products
# derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

function(_parse_arguments_with_repeated_keywords prefix _optionNames _singleArgNames _multiArgNames _repeatMultiArgNames)
  # first set all result variables to empty/FALSE
  foreach(arg_name ${_singleArgNames} ${_multiArgNames} ${_repeatSingleArgNames} ${_repeatMultiArgNames})
    set(${prefix}_${arg_name})
  endforeach(arg_name)

  foreach(option ${_optionNames})
    set(${prefix}_${option} FALSE)
  endforeach(option)

  set(${prefix}_UNPARSED_ARGUMENTS)

  set(insideValues FALSE)
  set(currentArgName)

  # now iterate over all arguments and fill the result variables
  foreach(currentArg ${ARGN})
    list(FIND _optionNames "${currentArg}" optionIndex)  # ... then this marks the end of the arguments belonging to this keyword
    list(FIND _singleArgNames "${currentArg}" singleArgIndex)  # ... then this marks the end of the arguments belonging to this keyword
    list(FIND _multiArgNames "${currentArg}" multiArgIndex)  # ... then this marks the end of the arguments belonging to this keyword
    list(FIND _repeatMultiArgNames "${currentArg}" repeatMultiArgIndex)  # ... then this marks the end of the arguments belonging to this keyword

    if(${optionIndex} EQUAL -1  AND  ${singleArgIndex} EQUAL -1  AND  ${multiArgIndex} EQUAL -1  AND  ${repeatMultiArgIndex} EQUAL -1)
      if(insideValues)
        if("${insideValues}" STREQUAL "SINGLE")
          set(${prefix}_${currentArgName} ${currentArg})
          set(insideValues FALSE)
        elseif("${insideValues}" STREQUAL "MULTI")
          list(APPEND ${prefix}_${currentArgName} ${currentArg})
        elseif("${insideValues}" STREQUAL "REPEAT")
          # get and pop last element from list
          list(REVERSE ${prefix}_${currentArgName})
          list(GET ${prefix}_${currentArgName} 0 _items)
          list(REMOVE_AT ${prefix}_${currentArgName} 0)
          list(REVERSE ${prefix}_${currentArgName})
          # append value to element
          set(_items "${_items} ${currentArg}")
          # append element as string to list
          list(APPEND ${prefix}_${currentArgName} "${_items}")
        endif()
      else(insideValues)
        list(APPEND ${prefix}_UNPARSED_ARGUMENTS ${currentArg})
      endif(insideValues)
    else()
      if(NOT ${optionIndex} EQUAL -1)
        set(${prefix}_${currentArg} TRUE)
        set(insideValues FALSE)
      elseif(NOT ${singleArgIndex} EQUAL -1)
        set(currentArgName ${currentArg})
        set(${prefix}_${currentArgName})
        set(insideValues "SINGLE")
      elseif(NOT ${multiArgIndex} EQUAL -1)
        set(currentArgName ${currentArg})
        set(${prefix}_${currentArgName})
        set(insideValues "MULTI")
      elseif(NOT ${repeatMultiArgIndex} EQUAL -1)
        set(currentArgName ${currentArg})
        # use space since empty items are not supported
        list(APPEND ${prefix}_${currentArgName} " ")
        set(insideValues "REPEAT")
      endif()
    endif()

  endforeach(currentArg)

  # propagate the result variables to the caller:
  foreach(arg_name ${_singleArgNames} ${_multiArgNames} ${_optionNames})
    set(${prefix}_${arg_name}  ${${prefix}_${arg_name}} PARENT_SCOPE)
  endforeach(arg_name)
  # strip spaces from the items of all repeated arguments
  foreach(arg_name ${_repeatMultiArgNames})
    set(stripped "")
    foreach(items ${${prefix}_${arg_name}})
      string(STRIP "${items}" items)
      list(APPEND stripped "${items}")
    endforeach()
    set(${prefix}_${arg_name}  ${stripped} PARENT_SCOPE)
  endforeach(arg_name)
  set(${prefix}_UNPARSED_ARGUMENTS ${${prefix}_UNPARSED_ARGUMENTS} PARENT_SCOPE)

endfunction()