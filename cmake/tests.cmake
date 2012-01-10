macro(initialize_tests)
  if(NOT TARGET tests)
    message("TODO: use rostest-check-results to verify that result files were generated")
    add_custom_target(tests)
  endif()
  if(NOT TARGET test)
    add_custom_target(test)
    add_dependencies(test tests)
  endif()

  # TODO: find rosunit somehow
  find_program(ROSUNIT_EXE rosunit
    PATHS ${CMAKE_SOURCE_DIR}/ros/tools/rosunit/scripts
    NO_DEFAULT_PATH)

  # TODO: [tds] rosunit should define these macros, not us.  I guess.
  find_program(ROSUNIT_SUMMARIZE_EXE summarize_results.py
    PATHS ${CMAKE_SOURCE_DIR}/ros/tools/rosunit/scripts
    NO_DEFAULT_PATH)
  find_program(ROSUNIT_CHECK_TEST_RAN_EXE check_test_ran.py
    PATHS ${CMAKE_SOURCE_DIR}/ros/tools/rosunit/scripts
    NO_DEFAULT_PATH)

  if(NOT TARGET clean-test-results)
    # Clean out previous test results before running tests.
    add_custom_target(clean-test-results
      COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/test_results
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
      ${ROSUNIT_SUMMARIZE_EXE} --nodeps ${PROJECT_NAME})
    add_dependencies(test ${PROJECT_NAME}_run_tests)
    add_dependencies(${PROJECT_NAME}_run_tests tests)
  endif()

  # gotcha:  you need a newline or the message doesn't appear
  # in the cache.   pfft.
  file(APPEND ${cachefile} "${ARGN}\n")
endmacro()

function(add_pyunit file)

  # Look for optional TIMEOUT argument, #2645
  parse_arguments(_pyunit "TIMEOUT;WORKING_DIRECTORY" "" ${ARGN})
  if(NOT _pyunit_TIMEOUT)
    set(_pyunit_TIMEOUT 60.0)
  endif()
  if(_pyunit_WORKING_DIRECTORY)
    set(_chdir_prefix "bash -c \"cd ${_pyunit_WORKING_DIRECTORY} &&")
    set(_chdir_suffix "\"")
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
  append_test_to_cache(catkin-tests "${_chdir_prefix} ${ROSUNIT_EXE} --name=${_testname} --time-limit=${_pyunit_TIMEOUT} --package=${PROJECT_NAME} -- ${_file_name} ${_covarg} ${_chdir_suffix}")
  append_test_to_cache(catkin-tests "${ROSUNIT_CHECK_TEST_RAN_EXE} ${CMAKE_BINARY_DIR}/test_results/${PROJECT_NAME}/TEST-${_testname}.xml")
endfunction()

function(add_gtest exe)

  # Look for optional TIMEOUT argument, #2645
  parse_arguments(_gtest "TIMEOUT;WORKING_DIRECTORY" "" ${ARGN})
  if(NOT _gtest_TIMEOUT)
    set(_gtest_TIMEOUT 60.0)
  endif()
  if(_gtest_WORKING_DIRECTORY)
    set(_chdir_prefix "bash -c \"cd ${_gtest_WORKING_DIRECTORY} && ")
    set(_chdir_suffix "\"")
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
  append_test_to_cache(catkin-tests "${_chdir_prefix}${ROSUNIT_EXE} --name=${_testname} --time-limit=${_gtest_TIMEOUT} --package=${PROJECT_NAME} ${_exe_path}/${exe}${_chdir_suffix}")
  append_test_to_cache(catkin-tests "${ROSUNIT_CHECK_TEST_RAN_EXE} ${CMAKE_BINARY_DIR}/test_results/${PROJECT_NAME}/TEST-${_testname}.xml")
endfunction()

function(add_nosetests dir)
  # Check that nosetests is installed.
  find_program(nosetests_path nosetests)
  if(NOT nosetests_path)
    message(FATAL_ERROR "Can't find nosetests executable")
  endif()

  parse_arguments(_nose "WORKING_DIRECTORY" "" ${ARGN})
  if(_nose_WORKING_DIRECTORY)
    set(_chdir_prefix "bash -c \"cd ${_nose_WORKING_DIRECTORY} && ")
    set(_chdir_suffix "\"")
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
  append_test_to_cache(catkin-tests "${CMAKE_COMMAND} -E make_directory ${output_dir_name}")
  string(REPLACE "/" "." output_file_name ${dir})
  append_test_to_cache(catkin-tests "${_chdir_prefix}${nosetests_path} --where=${_dir_name} --with-xunit --xunit-file=${output_dir_name}/${output_file_name}.xml ${_covarg}${_chdir_suffix}")
  append_test_to_cache(catkin-tests "${ROSUNIT_CHECK_TEST_RAN_EXE} ${output_dir_name}/${output_file_name}.xml")
endfunction()

function(add_rostest file)
  # Check that we can find rostest
  find_program(rostest_exe rostest PATHS ${CMAKE_BINARY_DIR}/bin)
  if(NOT rostest_exe)
    message(FATAL_ERROR "Can't find rostest executable")
  endif()

  parse_arguments(_rosunit "WORKING_DIRECTORY" "" ${ARGN})
  if(_rosunit_WORKING_DIRECTORY)
    set(_chdir_prefix "bash -c \"cd ${_rosunit_WORKING_DIRECTORY} && ")
    set(_chdir_suffix "\"")
  endif()

  # Check that the file exists, #1621
  set(_file_name _file_name-NOTFOUND)
  find_file(_file_name ${file}
            PATHS ${CMAKE_CURRENT_SOURCE_DIR}
            NO_DEFAULT_PATH)
  if(NOT _file_name)
    message(FATAL_ERROR "Can't find rostest file \"${file}\"")
  endif()

  append_test_to_cache(catkin-tests "${_chdir_prefix}${rostest_exe} ${_file_name}${_chdir_suffix}")
endfunction()
