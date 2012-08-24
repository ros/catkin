# set directory for test results
if(ENV{CATKIN_TEST_RESULTS_DIR})
  message("Override test results directory with environment variable CATKIN_TEST_RESULTS_DIR=${CATKIN_TEST_RESULTS_DIR}")
  set(CATKIN_TEST_RESULTS_DIR $ENV{CATKIN_TEST_RESULTS_DIR} CACHE INTERNAL "")
else()
  set(CATKIN_TEST_RESULTS_DIR ${CMAKE_BINARY_DIR}/test_results CACHE INTERNAL "")
endif()

# create target to build tests
if(NOT TARGET tests)
  add_custom_target(tests)
  message("TODO: implement add_roslaunch_check() in rostest-extras.cmake")
endif()

# create target to run tests
if(NOT TARGET run_tests)
  add_custom_target(run_tests)
  add_dependencies(run_tests tests)
endif()

# create target to clean test results
if(NOT TARGET clean-test-results)
  # clean out previous test results before running tests
  add_custom_target(clean-test-results
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CATKIN_TEST_RESULTS_DIR})
  # make the 'tests' target depend on 'clean-test-results'
  # which will ensure that test results are deleted before we try to build tests
  # and thus before we try to run tests
  add_dependencies(tests clean-test-results)
endif()

# all test results go under ${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME}/..
function(catkin_run_tests_target type name xunit_filename)
  parse_arguments(_testing "COMMAND;WORKING_DIRECTORY" "" ${ARGN})
  if(_testing_DEFAULT_ARGS)
    message(FATAL_ERROR "catkin_run_tests_target() called with unused arguments: ${_testing_DEFAULT_ARGS}")
  endif()

  set(command_file ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/catkin_tests/${PROJECT_NAME}.${type}.${name}.tests)
  set(results ${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME}/${xunit_filename})
  # requires newline at the end so that the message appears in the file
  file(WRITE ${command_file} "# '${type}' invocation in project '${PROJECT_NAME}' for tests '${name}'\n")
  file(APPEND ${command_file} "# results should be stored in '${results}'\n")
  file(APPEND ${command_file} "${_testing_COMMAND}\n")

  # create meta target to trigger all tests of a project
  if(NOT TARGET run_tests_${PROJECT_NAME})
    add_custom_target(run_tests_${PROJECT_NAME})
    add_dependencies(run_tests run_tests_${PROJECT_NAME})
  endif()
  # create meta target to trigger all tests of a specific type of a project
  if(NOT TARGET run_tests_${PROJECT_NAME}_${type})
    add_custom_target(run_tests_${PROJECT_NAME}_${type})
    add_dependencies(run_tests_${PROJECT_NAME} run_tests_${PROJECT_NAME}_${type})
  endif()
  # create target for test execution
  if (_testing_WORKING_DIRECTORY)
    set(working_dir_arg "--working-dir" ${_testing_WORKING_DIRECTORY})
  endif()
  assert(CATKIN_ENV)
  message(STATUS "env ${CATKIN_ENV}")
  add_custom_target(run_tests_${PROJECT_NAME}_${type}_${name}
    COMMAND ${CATKIN_ENV} echo $$ROS_PACKAGE_PATH)
    #${PYTHON_EXECUTABLE}
    #${catkin_EXTRAS_DIR}/test/run_tests.py --results ${results} ${working_dir_arg} ${_testing_COMMAND})
  add_dependencies(run_tests_${PROJECT_NAME}_${type} run_tests_${PROJECT_NAME}_${type}_${name})
  add_dependencies(run_tests_${PROJECT_NAME}_${type}_${name} tests)
endfunction()
