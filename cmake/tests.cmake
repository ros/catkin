macro(initialize_tests)
  if(NOT TARGET tests)
    add_custom_target(tests)
  endif()
  if(NOT TARGET test)
    add_custom_target(test)
    add_dependencies(test tests)
  endif()

  # TODO: find rosunit somehow
  set(rosunit_path ${CMAKE_SOURCE_DIR}/ros/tools/rosunit PARENT_SCOPE)
  set(rosunit_path ${CMAKE_SOURCE_DIR}/ros/tools/rosunit)

  if(NOT TARGET clean-test-results)
    # Clean out previous test results before running tests.
    add_custom_target(clean-test-results
      COMMAND cmake -E remove_directory ${CMAKE_BINARY_DIR}/test_results
      )

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
    set(${PROJECT_NAME}_CACHE TRUE)
    set(${PROJECT_NAME}_CACHE TRUE PARENT_SCOPE)
    # One test target per project
    add_custom_target(${PROJECT_NAME}_run_tests
      COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE}
      ${catkin_EXTRAS_DIR}/test/runtests.py ${cachefile}
      COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE}
      ${rosunit_path}/scripts/summarize_results.py --nodeps ${PROJECT_NAME})
    add_dependencies(test ${PROJECT_NAME}_run_tests)
    add_dependencies(${PROJECT_NAME}_run_tests tests)
  endif()

  # gotcha:  you need a newline or the message doesn't appear
  # in the cache.   pfft.
  file(APPEND ${cachefile} "${ARGN}\n")
endmacro()

function(add_pyunit file)
  # Check that we can find rosunit
  find_program(rosunit_exe rosunit)
  if(NOT rosunit_exe)
    message(FATAL_ERROR "Can't find rosunit executable")
  endif()

  # Look for optional TIMEOUT argument, #2645
  parse_arguments(_pyunit "TIMEOUT" "" ${ARGN})
  if(NOT _pyunit_TIMEOUT)
    set(_pyunit_TIMEOUT 60.0)
  endif()

  # Check that the file exists, #1621
  set(_file_name _file_name-NOTFOUND)
  find_file(_file_name ${file} 
            PATHS ${CMAKE_CURRENT_SOURCE_DIR} 
            NO_DEFAULT_PATH)
  if(NOT _file_name)
    message(FATAL_ERROR "Can't find pyunit file \"${file}\"")
  endif()

  # We look for ROS_TEST_COVERAGE=1
  # to indicate that coverage reports are being requested.
  if("$ENV{ROS_TEST_COVERAGE}" STREQUAL "1")
    set(_covarg "--cov")
  else()
    set(_covarg)
  endif()

  # Create a legal test name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname ${file})
  # We use rostest to call the executable to get process control, #1629
  append_test_to_cache(catkin-tests "${rosunit_exe} --name=${_testname} --time-limit=${_pyunit_TIMEOUT} --package=${PROJECT_NAME} -- ${_file_name} ${_covarg}")
endfunction()

function(add_gtest exe)
  # Check that we can find rosunit
  find_program(rosunit_exe rosunit)
  if(NOT rosunit_exe)
    message(FATAL_ERROR "Can't find rosunit executable")
  endif()

  # Look for optional TIMEOUT argument, #2645
  parse_arguments(_gtest "TIMEOUT" "" ${ARGN})
  if(NOT _gtest_TIMEOUT)
    set(_gtest_TIMEOUT 60.0)
  endif()

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
  append_test_to_cache(catkin-tests "${rosunit_exe} --name=${_testname} --time-limit=${_gtest_TIMEOUT} --package=${PROJECT_NAME} ${_exe_path}/${exe}")
endfunction()

function(add_nosetests dir)
  # Check that nosetests is installed.
  find_program(nosetests_path nosetests)
  if(NOT nosetests_path)
    message(FATAL_ERROR "Can't find nosetests executable")
  endif()

  # Check that the directory exists
  set(_dir_name _dir_name-NOTFOUND)
  find_file(_dir_name ${dir} 
            PATHS ${CMAKE_CURRENT_SOURCE_DIR} 
            NO_DEFAULT_PATH)
  if(NOT _dir_name)
    message(FATAL_ERROR "Can't find nosetests dir \"${dir}\"")
  endif()

  # We look for ROS_TEST_COVERAGE=1
  # to indicate that coverage reports are being requested.
  if("$ENV{ROS_TEST_COVERAGE}" STREQUAL "1")
    set(_covarg "--with-coverage")
  else()
    set(_covarg)
  endif()

  set(output_dir_name ${CMAKE_BINARY_DIR}/test_results/${PROJECT_NAME})
  append_test_to_cache(catkin-tests "cmake -E make_directory ${output_dir_name}")
  string(REPLACE "/" "." output_file_name ${dir})
  append_test_to_cache(catkin-tests "${nosetests_path} --where=${_dir_name} --with-xunit --xunit-file=${output_dir_name}/${output_file_name}.xml ${_covarg}")
endfunction()
