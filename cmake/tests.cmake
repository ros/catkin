macro(initialize_tests)
  if(NOT TARGET tests)
    add_custom_target(tests)
  endif()
  if(NOT TARGET test)
    add_custom_target(test)
    add_dependencies(test tests)
  endif()

  # We set the path variables both locally and in the parent scope so that
  # we can use them here and below in the add_* functions.
  # TODO: find catkin somehow
  set(catkin_path ${CMAKE_SOURCE_DIR}/catkin PARENT_SCOPE)
  set(catkin_path ${CMAKE_SOURCE_DIR}/catkin)
  # TODO: find rosunit somehow
  set(rosunit_path ${CMAKE_SOURCE_DIR}/ros/tools/rosunit PARENT_SCOPE)
  set(rosunit_path ${CMAKE_SOURCE_DIR}/ros/tools/rosunit)

  # Record where we're going to put test results (#2003)
  execute_process(COMMAND ${rosunit_path}/scripts/test_results_dir.py
                  OUTPUT_VARIABLE rosbuild_test_results_dir
                  RESULT_VARIABLE _test_results_dir_failed
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(_test_results_dir_failed)
    message(FATAL_ERROR "Failed to invoke ${rosunit_path}/scripts/test_results_dir.py")
  endif(_test_results_dir_failed)

  if(NOT TARGET clean-test-results)
    # Clean out previous test results before running tests.  Use bash
    # conditional to ignore failures (most often happens when a stale NFS
    # handle lingers in the test results directory), because CMake doesn't
    # seem to be able to do it.
    add_custom_target(clean-test-results
                      COMMAND if ! rm -rf ${rosbuild_test_results_dir}\; then echo "WARNING: failed to remove test-results directory"\; fi)
    # Make the tests target depend on clean-test-results, which will ensure
    # that test results are deleted before we try to build tests, and thus
    # before we try to run tests.
    add_dependencies(tests clean-test-results)
  endif()
endmacro()

initialize_tests()

# This is a macro to ensure that ${PROJECT_NAME}_CACHE gets set in a higher
# scope where we can check it later.
macro(append_test_to_cache CACHENAME)
  set(cachefile ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}.${CACHENAME})
  if(NOT ${PROJECT_NAME}_CACHE)
    file(WRITE ${cachefile} "#\n# This is ${cachefile}\n#\n")
    # Write out the directory that we'll use as CWD later when running
    # tests
    file(APPEND ${cachefile} "${PROJECT_SOURCE_DIR}\n")
    set(${PROJECT_NAME}_CACHE TRUE PARENT_SCOPE)
    # One test target per project
    add_custom_target(${PROJECT_NAME}_run_tests
                      COMMAND ${catkin_path}/scripts/runtests.py ${cachefile}
                      COMMAND ${rosunit_path}/scripts/summarize_results.py --nodeps ${PROJECT_NAME})
    add_dependencies(test ${PROJECT_NAME}_run_tests)
    add_dependencies(${PROJECT_NAME}_run_tests tests)
  endif()

  # gotcha:  you need a newline or the message doesn't appear
  # in the cache.   pfft.
  file(APPEND ${cachefile} "${ARGN}\n")
endmacro()

function(add_pyunit file)
  # Look for optional TIMEOUT argument, #2645
  parse_arguments(_pyunit "TIMEOUT" "" ${ARGN})
  if(NOT _pyunit_TIMEOUT)
    set(_pyunit_TIMEOUT 60.0)
  endif(NOT _pyunit_TIMEOUT)

  # Check that the file exists, #1621
  set(_file_name _file_name-NOTFOUND)
  find_file(_file_name ${file} ${PROJECT_SOURCE_DIR})
  if(NOT _file_name)
    message(FATAL_ERROR "Can't find pyunit file \"${file}\"")
  endif(NOT _file_name)

  # We look for ROS_TEST_COVERAGE=1
  # to indicate that coverage reports are being requested.
  if("$ENV{ROS_TEST_COVERAGE}" STREQUAL "1")
    set(_covarg "--cov")
  else("$ENV{ROS_TEST_COVERAGE}" STREQUAL "1")
    set(_covarg)
  endif("$ENV{ROS_TEST_COVERAGE}" STREQUAL "1")

  # Create a legal test name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname ${file})
  # We use rostest to call the executable to get process control, #1629
  append_test_to_cache(catkin-tests "${rosunit_path}/bin/rosunit --name=${_testname} --time-limit=${_pyunit_TIMEOUT} --package=${PROJECT_NAME} -- ${_file_name} ${_covarg}")
endfunction(add_pyunit)

function(add_gtest exe)
  # Look for optional TIMEOUT argument, #2645
  parse_arguments(_gtest "TIMEOUT" "" ${ARGN})
  if(NOT _gtest_TIMEOUT)
    set(_gtest_TIMEOUT 60.0)
  endif(NOT _gtest_TIMEOUT)

  # Create the program, with basic + gtest build flags
  find_package(GTest REQUIRED)
  include_directories(${GTEST_INCLUDE_DIRS})
  add_executable(${exe} EXCLUDE_FROM_ALL ${_gtest_DEFAULT_ARGS})
  target_link_libraries(${exe} ${GTEST_LIBRARIES} ${THREADS_LIBRARY})

  # Make sure the executable is built before running tests
  add_dependencies(tests ${exe})

  # Create a legal test name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname ${exe})
  get_target_property(_exe_path ${exe} RUNTIME_OUTPUT_DIRECTORY)
  # We use rosunit to call the executable to get process control, #1629, #3112
  append_test_to_cache(catkin-tests "${rosunit_path}/bin/rosunit --name=${_testname} --time-limit=${_gtest_TIMEOUT} --package=${PROJECT_NAME} ${_exe_path}/${exe}")
endfunction()
