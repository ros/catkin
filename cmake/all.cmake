# prevent multiple inclusion
if(_CATKIN_ALL_INCLUDED_)
  return()
endif()
set(_CATKIN_ALL_INCLUDED_ TRUE)

if(NOT catkin_BUILD_PREFIX)
  message(FATAL_ERROR "\ncatkin_BUILD_PREFIX is not set\n")
endif()
if(NOT catkin_EXTRAS_DIR)
  message(FATAL_ERROR "\ncatkin_EXTRAS_DIR is not set\n")
endif()

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

list(APPEND CMAKE_MODULE_PATH ${catkin_EXTRAS_DIR}/Modules)

# XXX move stuff to separate folders
# functions/macros: list_append_unique, safe_execute_process
# python-integration: catkin_python_setup.cmake, interrogate_setup_dot_py.py, templates/__init__.py.in, templates/script.py.in, templates/python_distutils_install.bat.in, templates/python_distutils_install.sh.in, templates/safe_execute_install.cmake.in
foreach(filename
    assert
    catkin_add_env_hooks
    catkin_generate_environment
    catkin_project
    catkin_stack
    catkin_workspace
    debug_message
    em_expand
    python # defines PYTHON_EXECUTABLE, required by empy
    empy
    find_program_required
    install_matching_to_share
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
    tools/libraries
    tools/rt

#    rosbuild_compat
#    tools/doxygen
#    tools/threads
  )
  include(${catkin_EXTRAS_DIR}/${filename}.cmake)
endforeach()

# undefine CATKIN_ENV since it might be set in the cache from a previous build
set(CATKIN_ENV "" CACHE INTERNAL "catkin environment" FORCE)

# generate environment files like env.* and setup.*
# uses em_expand without CATKIN_ENV being set yet
catkin_generate_environment()

# environment to call external processes
if(CMAKE_HOST_UNIX) # true for linux, apple, mingw-cross and cygwin
  set(CATKIN_ENV ${catkin_BUILD_PREFIX}/env.sh CACHE INTERNAL "catkin environment")
else()
  set(CATKIN_ENV ${catkin_BUILD_PREFIX}/env.bat CACHE INTERNAL "catkin environment")
endif()

# add additional environment hooks
catkin_add_env_hooks(05.catkin-test-results SHELLS bat sh DIRECTORY ${catkin_EXTRAS_DIR}/env-hooks)

foreach(filename
    catkin_python_setup # requires stamp and environment files
  )
  include(${catkin_EXTRAS_DIR}/${filename}.cmake)
endforeach()
