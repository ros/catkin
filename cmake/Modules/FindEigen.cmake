# - Try to find Eigen3 lib
#
# This module supports requiring a minimum version, e.g. you can do
#   find_package(Eigen3 3.1.2)
# to require version 3.1.2 or newer of Eigen3.
#
# Once done this will define
#
#  Eigen_FOUND - system has eigen lib with correct version
#  Eigen_INCLUDE_DIR - the eigen include directory
#  Eigen_VERSION - eigen version

# Copyright (c) 2006, 2007 Montel Laurent, <montel@kde.org>
# Copyright (c) 2008, 2009 Gael Guennebaud, <g.gael@free.fr>
# Copyright (c) 2009 Benoit Jacob <jacob.benoit.1@gmail.com>
# Redistribution and use is allowed according to the terms of the 2-clause BSD license.

if(NOT Eigen_FIND_VERSION)
#  if(NOT Eigen_FIND_VERSION_MAJOR)
#    set(Eigen_FIND_VERSION_MAJOR 3)
#  endif(NOT Eigen_FIND_VERSION_MAJOR)
#  if(NOT Eigen_FIND_VERSION_MINOR)
#    set(Eigen_FIND_VERSION_MINOR 91)
#  endif(NOT Eigen_FIND_VERSION_MINOR)
#  if(NOT Eigen_FIND_VERSION_PATCH)
#    set(Eigen_FIND_VERSION_PATCH 0)
#  endif(NOT Eigen_FIND_VERSION_PATCH)

  set(Eigen_FIND_VERSION "${Eigen_FIND_VERSION_MAJOR}.${Eigen_FIND_VERSION_MINOR}.${Eigen_FIND_VERSION_PATCH}")
endif(NOT Eigen_FIND_VERSION)

macro(_eigen3_check_version)
  file(READ "${Eigen_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h" _eigen3_version_header)

  string(REGEX MATCH "define[ \t]+Eigen_WORLD_VERSION[ \t]+([0-9]+)" _eigen3_world_version_match "${_eigen3_version_header}")
  set(Eigen_WORLD_VERSION "${CMAKE_MATCH_1}")
  string(REGEX MATCH "define[ \t]+Eigen_MAJOR_VERSION[ \t]+([0-9]+)" _eigen3_major_version_match "${_eigen3_version_header}")
  set(Eigen_MAJOR_VERSION "${CMAKE_MATCH_1}")
  string(REGEX MATCH "define[ \t]+Eigen_MINOR_VERSION[ \t]+([0-9]+)" _eigen3_minor_version_match "${_eigen3_version_header}")
  set(Eigen_MINOR_VERSION "${CMAKE_MATCH_1}")

  set(Eigen_VERSION ${Eigen_WORLD_VERSION}.${Eigen_MAJOR_VERSION}.${Eigen_MINOR_VERSION})
  if(${Eigen_VERSION} VERSION_LESS ${Eigen_FIND_VERSION})
    set(Eigen_VERSION_OK FALSE)
  else(${Eigen_VERSION} VERSION_LESS ${Eigen_FIND_VERSION})
    set(Eigen_VERSION_OK TRUE)
  endif(${Eigen_VERSION} VERSION_LESS ${Eigen_FIND_VERSION})

  if(NOT Eigen_VERSION_OK)

    message(STATUS "Eigen version ${Eigen_VERSION} found in ${Eigen_INCLUDE_DIR}, "
                   "but at least version ${Eigen_FIND_VERSION} is required")
  endif(NOT Eigen_VERSION_OK)
endmacro(_eigen3_check_version)

if (Eigen_INCLUDE_DIRS)

  # in cache already
  _eigen3_check_version()
  set(Eigen_FOUND ${Eigen_VERSION_OK})

else ()

  find_path(Eigen_INCLUDE_DIR NAMES signature_of_eigen3_matrix_library
      PATHS
      ${CMAKE_INSTALL_PREFIX}/include
      ${KDE4_INCLUDE_DIR}
      PATH_SUFFIXES eigen3 eigen
    )

  if(Eigen_INCLUDE_DIR)
    _eigen3_check_version()
  endif(Eigen_INCLUDE_DIR)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(Eigen DEFAULT_MSG Eigen_INCLUDE_DIR Eigen_VERSION_OK)

  mark_as_advanced(Eigen_INCLUDE_DIR)
  SET(Eigen_INCLUDE_DIRS ${Eigen_INCLUDE_DIR} CACHE PATH "The Eigen include path.")

endif()

