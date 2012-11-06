# prevent multiple inclusion
if(DEFINED _CATKIN_ALL_INCLUDED_)
  message(FATAL_ERROR "catkin/cmake/all.cmake included multiple times")
endif()
set(_CATKIN_ALL_INCLUDED_ TRUE)

if(NOT DEFINED catkin_EXTRAS_DIR)
  message(FATAL_ERROR "catkin_EXTRAS_DIR is not set")
endif()

# define devel space
if(CATKIN_DEVEL_PREFIX)
  set(CATKIN_DEVEL_PREFIX ${CATKIN_DEVEL_PREFIX} CACHE PATH "catkin devel space")
else()
  set(CATKIN_DEVEL_PREFIX "${CMAKE_BINARY_DIR}/develspace")
endif()
message(STATUS "Using CATKIN_DEVEL_PREFIX: ${CATKIN_DEVEL_PREFIX}")
# this variable is only for backward compatibility
set(CATKIN_BUILD_PREFIX "${CATKIN_DEVEL_PREFIX}")

# create workspace marker
set(_sourcespaces "${CMAKE_SOURCE_DIR}")
if(EXISTS "${CATKIN_DEVEL_PREFIX}/.catkin")
  # prepend to existing list of sourcespaces
  file(READ "${CATKIN_DEVEL_PREFIX}/.catkin" _existing_sourcespaces)
  list(FIND _existing_sourcespaces "${CMAKE_SOURCE_DIR}" _index)
  if(_index EQUAL -1)
    list(INSERT _existing_sourcespaces 0 ${CMAKE_SOURCE_DIR})
  endif()
  set(_sourcespaces ${_existing_sourcespaces})
endif()
file(WRITE "${CATKIN_DEVEL_PREFIX}/.catkin" "${_sourcespaces}")


# use either CMAKE_PREFIX_PATH explicitly passed to CMake as a command line argument
# or CMAKE_PREFIX_PATH from the environment
if(NOT DEFINED CMAKE_PREFIX_PATH)
  if(NOT "$ENV{CMAKE_PREFIX_PATH}" STREQUAL "")
    string(REPLACE ":" ";" CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
  endif()
endif()
if(CMAKE_PREFIX_PATH)
  # skip devel space if it is in CMAKE_PREFIX_PATH so that it is not part of CATKIN_WORKSPACES
  list(REMOVE_ITEM CMAKE_PREFIX_PATH ${CATKIN_DEVEL_PREFIX})
  message(STATUS "Using CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
endif()

# list of unique catkin workspaces based on CMAKE_PREFIX_PATH
set(CATKIN_WORKSPACES "")
foreach(path ${CMAKE_PREFIX_PATH})
  if(EXISTS "${path}/.catkin")
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
set(CMAKE_PREFIX_PATH_WITHOUT_DEVEL_SPACE ${CMAKE_PREFIX_PATH})

# prepend devel space to CMAKE_PREFIX_PATH
list(INSERT CMAKE_PREFIX_PATH 0 ${CATKIN_DEVEL_PREFIX})


# enable all new policies (if available)
macro(_set_cmake_policy_to_new_if_available policy)
  if(POLICY ${policy})
    cmake_policy(SET ${policy} NEW)
  endif()
endmacro()
_set_cmake_policy_to_new_if_available(CMP0000)
_set_cmake_policy_to_new_if_available(CMP0001)
_set_cmake_policy_to_new_if_available(CMP0002)
_set_cmake_policy_to_new_if_available(CMP0003)
_set_cmake_policy_to_new_if_available(CMP0004)
_set_cmake_policy_to_new_if_available(CMP0005)
_set_cmake_policy_to_new_if_available(CMP0006)
_set_cmake_policy_to_new_if_available(CMP0007)
_set_cmake_policy_to_new_if_available(CMP0008)
_set_cmake_policy_to_new_if_available(CMP0009)
_set_cmake_policy_to_new_if_available(CMP0010)
_set_cmake_policy_to_new_if_available(CMP0011)
_set_cmake_policy_to_new_if_available(CMP0012)
_set_cmake_policy_to_new_if_available(CMP0013)
_set_cmake_policy_to_new_if_available(CMP0014)
_set_cmake_policy_to_new_if_available(CMP0015)
_set_cmake_policy_to_new_if_available(CMP0016)
_set_cmake_policy_to_new_if_available(CMP0017)

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
    catkin_destinations
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
    list_insert_in_workspace_order
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
# ensure that no current package name is set
unset(_CATKIN_CURRENT_PACKAGE)

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
set(OUTPUT_SCRIPT_DIR ${CMAKE_BINARY_DIR}/catkin_generated)
set(PREPEND_SPACE_DIR ${CATKIN_DEVEL_PREFIX})
set(CUSTOM_PREFIX_PATH ${CMAKE_PREFIX_PATH})
em_expand(${catkin_EXTRAS_DIR}/templates/generate_cached_env.context.py.in
  ${CMAKE_BINARY_DIR}/catkin_generated/generate_cached_env.develspace.context.py
  ${catkin_EXTRAS_DIR}/em/generate_cached_env.py.em
  ${CMAKE_BINARY_DIR}/catkin_generated/generate_cached_env.py)
set(_command_option "")
if(CATKIN_STATIC_ENV)
  set(_command_option "--static")
endif()
set(GENERATE_ENVIRONMENT_CACHE_COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_BINARY_DIR}/catkin_generated/generate_cached_env.py ${_command_option})
# the script is generated once here and refreshed by every call to catkin_add_env_hooks()
safe_execute_process(COMMAND ${GENERATE_ENVIRONMENT_CACHE_COMMAND})
# environment to call external processes
set(CATKIN_ENV ${CMAKE_BINARY_DIR}/catkin_generated/env_cached.${script_ext} CACHE INTERNAL "catkin environment")

if(CATKIN_STATIC_ENV)
  # generate cached env script for installspace when requesting a static environment
  set(OUTPUT_SCRIPT_DIR ${CMAKE_BINARY_DIR}/catkin_generated/installspace)
  set(PREPEND_SPACE_DIR ${CMAKE_INSTALL_PREFIX})
  set(CUSTOM_PREFIX_PATH ${CATKIN_WORKSPACES})
  em_expand(${catkin_EXTRAS_DIR}/templates/generate_cached_env.context.py.in
    ${CMAKE_BINARY_DIR}/catkin_generated/generate_cached_env.installspace.context.py
    ${catkin_EXTRAS_DIR}/em/generate_cached_env.py.em
    ${CMAKE_BINARY_DIR}/catkin_generated/installspace/generate_cached_env.py)
  set(cmd ${PYTHON_EXECUTABLE} ${CMAKE_BINARY_DIR}/catkin_generated/installspace/generate_cached_env.py ${_command_option})
  safe_execute_process(COMMAND ${cmd})
  install(PROGRAMS ${OUTPUT_SCRIPT_DIR}/env_cached.${script_ext}
    DESTINATION .)
endif()

# add additional environment hooks
if(CATKIN_BUILD_BINARY_PACKAGE AND NOT "${PROJECT_NAME}" STREQUAL "catkin")
  set(catkin_skip_install_env_hooks "SKIP_INSTALL")
endif()
if(CMAKE_HOST_UNIX)
  catkin_add_env_hooks(05.catkin-test-results SHELLS sh DIRECTORY ${catkin_EXTRAS_DIR}/env-hooks ${catkin_skip_install_env_hooks})
else()
  catkin_add_env_hooks(05.catkin-test-results SHELLS bat DIRECTORY ${catkin_EXTRAS_DIR}/env-hooks ${catkin_skip_install_env_hooks})
endif()

# requires stamp and environment files
include(${catkin_EXTRAS_DIR}/catkin_python_setup.cmake)
