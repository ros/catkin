#
# Add a GTest based test target.
#
# An executable target is created with the source files, it is linked
# against GTest and added to the set of unit tests.
#
# .. note:: The test can be executed by calling the binary directly
#   or using: ``make run_tests_${PROJECT_NAME}_gtest_${target}``
#
# :param target: the target name
# :type target: string
# :param source_files: a list of source files used to build the test
#   executable
# :type source_files: list of strings
# :param WORKING_DIRECTORY: the working directory when executing the
#   executable
# :type FILE_PREFIX: string
# :param TIMEOUT: currently not supported
# :type FILE_PREFIX: integer
#
# @public
#
function(catkin_add_gtest target)
  if(NOT GTEST_FOUND)
    message(STATUS "skipping gtest '${target}' in project '${PROJECT_NAME}'")
    return()
  endif()

  # XXX look for optional TIMEOUT argument, #2645
  parse_arguments(_gtest "TIMEOUT;WORKING_DIRECTORY" "" ${ARGN})
  if(_gtest_TIMEOUT)
    message(WARNING "TIMEOUT argument to catkin_add_gtest() is ignored")
  endif()

  # create the executable, with basic + gtest build flags
  include_directories(${GTEST_INCLUDE_DIRS})
  link_directories(${GTEST_LIBRARY_DIRS})
  add_executable(${target} EXCLUDE_FROM_ALL ${_gtest_DEFAULT_ARGS})
  target_link_libraries(${target} ${GTEST_LIBRARIES} ${THREADS_LIBRARY})

  # make sure the target is built before running tests
  add_dependencies(tests ${target})

  # XXX we DONT use rosunit to call the executable to get process control, #1629, #3112
  get_target_property(_target_path ${target} RUNTIME_OUTPUT_DIRECTORY)
  set(cmd "${_target_path}/${target} --gtest_output=xml:${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME}/gtest-${target}.xml")
  catkin_run_tests_target("gtest" ${target} "gtest-${target}.xml" COMMAND ${cmd} WORKING_DIRECTORY ${_gtest_WORKING_DIRECTORY})
endfunction()

function(add_gtest)
  message(WARNING "add_gtest() is deprecated, please rename the function call to catkin_add_gtest()")
  catkin_add_gtest(${ARGN})
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
