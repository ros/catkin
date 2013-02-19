# allow overriding CATKIN_TEST_RESULTS_DIR when explicitly passed to CMake as a command line argument
if(DEFINED CATKIN_TEST_RESULTS_DIR)
  set(CATKIN_TEST_RESULTS_DIR ${CATKIN_TEST_RESULTS_DIR} CACHE INTERNAL "")
else()
  set(CATKIN_TEST_RESULTS_DIR ${CMAKE_BINARY_DIR}/test_results CACHE INTERNAL "")
endif()
message(STATUS "Using CATKIN_TEST_RESULTS_DIR: ${CATKIN_TEST_RESULTS_DIR}")
file(MAKE_DIRECTORY ${CATKIN_TEST_RESULTS_DIR})

# create target to build tests
if(NOT TARGET tests)
  add_custom_target(tests)
endif()

# create target to run all tests
# it uses the dot-prefixed test targets to depend on building all tests and cleaning test results before the tests are executed
if(NOT TARGET run_tests)
  add_custom_target(run_tests)
endif()

# create target to clean test results
if(NOT TARGET clean_test_results)
  add_custom_target(clean_test_results
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CATKIN_TEST_RESULTS_DIR})
endif()

#
# Create a test target, integrate it with the run_tests infrastructure
# and post-process the junit result.
#
# All test results go under ${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME}/..
#
# This function is only used internally by the various
# catkin_add_*test() functions.
#
function(catkin_run_tests_target type name xunit_filename)
  cmake_parse_arguments(_testing "" "WORKING_DIRECTORY" "COMMAND;DEPENDENCIES" ${ARGN})
  if(_testing_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "catkin_run_tests_target() called with unused arguments: ${_testing_UNPARSED_ARGUMENTS}")
  endif()

  # create meta target to trigger all tests of a project
  if(NOT TARGET run_tests_${PROJECT_NAME})
    add_custom_target(run_tests_${PROJECT_NAME})
    # create hidden meta target which depends on hidden test targets which depend on clean_test_results
    add_custom_target(.run_tests_${PROJECT_NAME})
    # run_tests depends on this hidden target hierarchy to clear test results before running all tests
    add_dependencies(run_tests .run_tests_${PROJECT_NAME})
  endif()
  # create meta target to trigger all tests of a specific type of a project
  if(NOT TARGET run_tests_${PROJECT_NAME}_${type})
    add_custom_target(run_tests_${PROJECT_NAME}_${type})
    add_dependencies(run_tests_${PROJECT_NAME} run_tests_${PROJECT_NAME}_${type})
    # hidden meta target which depends on hidden test targets which depend on clean_test_results
    add_custom_target(.run_tests_${PROJECT_NAME}_${type})
    add_dependencies(.run_tests_${PROJECT_NAME} .run_tests_${PROJECT_NAME}_${type})
  endif()
  # create target for test execution
  set(results ${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME}/${xunit_filename})
  if (_testing_WORKING_DIRECTORY)
    set(working_dir_arg "--working-dir" ${_testing_WORKING_DIRECTORY})
  endif()
  assert(CATKIN_ENV)
  set(cmd ${CATKIN_ENV} ${PYTHON_EXECUTABLE}
    ${catkin_EXTRAS_DIR}/test/run_tests.py ${results} ${working_dir_arg} ${_testing_COMMAND})
  add_custom_target(run_tests_${PROJECT_NAME}_${type}_${name}
    COMMAND ${cmd})
  add_dependencies(run_tests_${PROJECT_NAME}_${type} run_tests_${PROJECT_NAME}_${type}_${name})
  if(_testing_DEPENDENCIES)
    add_dependencies(run_tests_${PROJECT_NAME}_${type}_${name} ${_testing_DEPENDENCIES})
  endif()
  # hidden test target which depends on building all tests and cleaning test results
  add_custom_target(.run_tests_${PROJECT_NAME}_${type}_${name}
    COMMAND ${cmd})
  add_dependencies(.run_tests_${PROJECT_NAME}_${type} .run_tests_${PROJECT_NAME}_${type}_${name})
  add_dependencies(.run_tests_${PROJECT_NAME}_${type}_${name} clean_test_results tests ${_testing_DEPENDENCIES})
endfunction()
