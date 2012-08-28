#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

function(add_gtest exe)
  if(NOT GTEST_FOUND)
    message(STATUS "skipping gtest '${exe}' in project '${PROJECT_NAME}'")
    return()
  endif()

  # XXX look for optional TIMEOUT argument, #2645
  parse_arguments(_gtest "TIMEOUT;WORKING_DIRECTORY" "" ${ARGN})
  if(_gtest_TIMEOUT)
    message(WARNING "TIMEOUT argument to add_gtest() is ignored")
  endif()

  # create the executable, with basic + gtest build flags
  include_directories(${GTEST_INCLUDE_DIRS})
  link_directories(${GTEST_LIBRARY_DIRS})
  add_executable(${exe} EXCLUDE_FROM_ALL ${_gtest_DEFAULT_ARGS})
  target_link_libraries(${exe} ${GTEST_LIBRARIES} ${THREADS_LIBRARY})

  # make sure the executable is built before running tests
  add_dependencies(tests ${exe})

  # sanitize test name to be used as a legal target name
  string(REPLACE "/" "_" _testname ${exe})
  # XXX we DONT use rosunit to call the executable to get process control, #1629, #3112
  get_target_property(_exe_path ${exe} RUNTIME_OUTPUT_DIRECTORY)
  set(cmd "${_exe_path}/${exe} --gtest_output=xml:${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME}/gtest-${_testname}.xml")
  catkin_run_tests_target("gtest" ${_testname} "gtest-${_testname}.xml" COMMAND ${cmd} WORKING_DIRECTORY ${_gtest_WORKING_DIRECTORY})
endfunction()

if(_CATKIN_GTEST_SRC_FOUND)
  return()
endif()

find_package(GTest QUIET)
if(NOT GTEST_FOUND)
  find_file(_CATKIN_GTEST_SRC "gtest.cc" PATHS "/usr/src/gtest/src" "${CMAKE_SOURCE_DIR}/gtest/src" NO_DEFAULT_PATH NO_CMAKE_PATH)

  if(_CATKIN_GTEST_SRC)
    GET_FILENAME_COMPONENT(_CATKIN_GTEST_DIR ${_CATKIN_GTEST_SRC} PATH)
    set(_CATKIN_GTEST_DIR "${_CATKIN_GTEST_DIR}/../" CACHE INTERNAL "")
    # add source dir directly (this only works on Ubuntu for now)
    add_subdirectory(${_CATKIN_GTEST_DIR} "../gtest")
    # set the same variables as find_package()
    set(GTEST_FOUND TRUE CACHE INTERNAL "")
    find_path(GTEST_INCLUDE_DIRS gtest/gtest.h)
    set(GTEST_INCLUDE_DIRS ${GTEST_INCLUDE_DIRS} CACHE INTERNAL "")
    set(GTEST_LIBRARIES "gtest" CACHE INTERNAL "")
    set(GTEST_MAIN_LIBRARIES "gtest_main" CACHE INTERNAL "")
    set(GTEST_BOTH_LIBRARIES "${GTEST_LIBRARIES};${GTEST_MAIN_LIBRARIES}" CACHE INTERNAL "")
    set(GTEST_LIBRARY_DIRS ${CMAKE_BINARY_DIR}/gtest CACHE INTERNAL "")
    message(STATUS "Found gtest sources under '/usr/src/gtest': gtests will be built")
    set(_CATKIN_GTEST_SRC_FOUND TRUE CACHE INTERNAL "")
  endif()
  if(NOT GTEST_FOUND)
    message(WARNING "gtest not found, C++ tests can not be built. You can run \"svn checkout http://googletest.googlecode.com/svn/tags/release-1.6.0 gtest\" in the root of your workspace")
  endif()
else()
  message(STATUS "Found gtest: gtests will be built")
  set(GTEST_FOUND ${GTEST_FOUND} CACHE INTERNAL "")
  set(GTEST_INCLUDE_DIRS ${GTEST_INCLUDE_DIRS} CACHE INTERNAL "")
  set(GTEST_LIBRARIES ${GTEST_LIBRARIES} CACHE INTERNAL "")
  set(GTEST_MAIN_LIBRARIES ${GTEST_MAIN_LIBRARIES} CACHE INTERNAL "")
  set(GTEST_BOTH_LIBRARIES ${GTEST_BOTH_LIBRARIES} CACHE INTERNAL "")
endif()
