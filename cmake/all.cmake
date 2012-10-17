find_package(catkin QUIET)
if(DEFINED catkin_VERSION)
  message(STATUS "Using catkin ${catkin_VERSION}")
else()
  message(STATUS "Using catkin from source")
endif()

# prevent multiple inclusion
if(DEFINED _CATKIN_ALL_INCLUDED_)
  message(FATAL_ERROR "catkin/cmake/all.cmake included multiple times")
endif()
set(_CATKIN_ALL_INCLUDED_ TRUE)

if(NOT DEFINED catkin_EXTRAS_DIR)
  message(FATAL_ERROR "catkin_EXTRAS_DIR is not set")
endif()

# use either CMAKE_PREFIX_PATH explicitly passed to CMake as a command line argument
# or CMAKE_PREFIX_PATH from the environment
if(NOT DEFINED CMAKE_PREFIX_PATH)
  if(NOT "$ENV{CMAKE_PREFIX_PATH}" STREQUAL "")
    string(REPLACE ":" ";" CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
  endif()
endif()
if(CMAKE_PREFIX_PATH)
  message(STATUS "Using CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
endif()

# list of unique catkin workspaces based on CMAKE_PREFIX_PATH
set(CATKIN_WORKSPACES "")
foreach(path ${CMAKE_PREFIX_PATH})
  if(EXISTS "${path}/.CATKIN_WORKSPACE")
    list(FIND CATKIN_WORKSPACES ${path} _index)
    if(_index EQUAL -1)
      list(APPEND CATKIN_WORKSPACES ${path})
    endif()
  endif()
endforeach()
if(CATKIN_WORKSPACES)
  message(STATUS "This workspace overlays: ${CATKIN_WORKSPACES}")
endif()

# save original CMAKE_PREFIX_PATH for environment generation
set(CMAKE_PREFIX_PATH_WITHOUT_BUILDSPACE ${CMAKE_PREFIX_PATH})

# define buildspace
set(CATKIN_BUILD_PREFIX "${CMAKE_BINARY_DIR}/buildspace")
# prepend buildspace to CMAKE_PREFIX_PATH
list(INSERT CMAKE_PREFIX_PATH 0 ${CATKIN_BUILD_PREFIX})


# enable all new policies
cmake_policy(SET CMP0000 NEW)
cmake_policy(SET CMP0001 NEW)
cmake_policy(SET CMP0002 NEW)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0004 NEW)
cmake_policy(SET CMP0005 NEW)
cmake_policy(SET CMP0006 NEW)
cmake_policy(SET CMP0007 NEW)
cmake_policy(SET CMP0008 NEW)
cmake_policy(SET CMP0009 NEW)
cmake_policy(SET CMP0010 NEW)
cmake_policy(SET CMP0011 NEW)
cmake_policy(SET CMP0012 NEW)
cmake_policy(SET CMP0013 NEW)
cmake_policy(SET CMP0014 NEW)
cmake_policy(SET CMP0015 NEW)
cmake_policy(SET CMP0016 NEW)
cmake_policy(SET CMP0017 NEW)

# the following operations must be performed inside a project context
if(NOT PROJECT_NAME)
  project(catkin_internal)
endif()

# include CMake functions
include(CMakeParseArguments)

# functions/macros: list_append_unique, safe_execute_process
# python-integration: catkin_python_setup.cmake, interrogate_setup_dot_py.py, templates/__init__.py.in, templates/script.py.in, templates/python_distutils_install.bat.in, templates/python_distutils_install.sh.in, templates/safe_execute_install.cmake.in
foreach(filename
    assert
    catkin_add_env_hooks
    catkin_generate_environment
    catkin_package
    catkin_package_xml
    catkin_workspace
    debug_message
    em_expand
    python # defines PYTHON_EXECUTABLE, required by empy
    empy
    find_program_required
    list_append_unique
    parse_arguments
    safe_execute_process
    stamp
    platform/lsb
    platform/ubuntu
    platform/windows
    test/download_test_data
    test/gtest
    test/nosetests
    test/tests
    tools/doxygen
    tools/libraries
    tools/rt

#    tools/threads
  )
  include(${catkin_EXTRAS_DIR}/${filename}.cmake)
endforeach()

# output catkin version for debugging
_catkin_package_xml(${CMAKE_BINARY_DIR}/catkin/catkin_generated/version DIRECTORY ${catkin_EXTRAS_DIR}/..)
message(STATUS "catkin ${catkin_VERSION}")

# set global install destinations
set(CATKIN_GLOBAL_BIN_DESTINATION bin)
set(CATKIN_GLOBAL_ETC_DESTINATION etc)
set(CATKIN_GLOBAL_INCLUDE_DESTINATION include)
set(CATKIN_GLOBAL_LIB_DESTINATION lib)
set(CATKIN_GLOBAL_LIBEXEC_DESTINATION lib)
set(CATKIN_GLOBAL_PYTHON_DESTINATION ${PYTHON_INSTALL_DIR})
set(CATKIN_GLOBAL_SHARE_DESTINATION share)

# undefine CATKIN_ENV since it might be set in the cache from a previous build
set(CATKIN_ENV "" CACHE INTERNAL "catkin environment" FORCE)

# generate environment files like env.* and setup.*
# uses em_expand without CATKIN_ENV being set yet
catkin_generate_environment()

# file extension of env script
if(CMAKE_HOST_UNIX) # true for linux, apple, mingw-cross and cygwin
  set(script_ext sh)
else()
  set(script_ext bat)
endif()
if(CATKIN_STATIC_ENV)
  # static environment avoid to call env script at all and uses current environment including the knowledge about the effects of the local setup.sh without evaluating it
  message(STATUS "Generating static environment")
  set(CATKIN_STATIC_ENV TRUE CACHE BOOL "Generate static environment")
endif()
# take snapshot of the modifications the env script causes
# to reproduce the same changes with a static script in a fraction of the time
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/catkin_generated/tmp)
em_expand(${catkin_EXTRAS_DIR}/templates/env_cached.context.py.in
  ${CMAKE_BINARY_DIR}/catkin_generated/env_cached.buildspace.context.py
  ${catkin_EXTRAS_DIR}/em/env_cached.em
  ${CMAKE_BINARY_DIR}/catkin_generated/tmp/env_cached.${script_ext})
file(COPY ${CMAKE_BINARY_DIR}/catkin_generated/tmp/env_cached.${script_ext}
  DESTINATION ${CMAKE_BINARY_DIR}/catkin_generated
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
# environment to call external processes
set(CATKIN_ENV ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/env_cached.${script_ext} CACHE INTERNAL "catkin environment")

# add additional environment hooks
if(CATKIN_BUILD_BINARY_PACKAGE AND NOT "${PROJECT_NAME}" STREQUAL "catkin")
  set(catkin_skip_install_env_hooks "SKIP_INSTALL")
endif()
catkin_add_env_hooks(05.catkin-test-results SHELLS bat sh DIRECTORY ${catkin_EXTRAS_DIR}/env-hooks ${catkin_skip_install_env_hooks})

foreach(filename
    catkin_python_setup # requires stamp and environment files
  )
  include(${catkin_EXTRAS_DIR}/${filename}.cmake)
endforeach()
