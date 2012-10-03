#
# Add Python nose tests.
#
# Nose collects tests from the directory ``dir`` automatically.
#
# .. note:: The test can be executed by calling ``nosetests``
#   directly or using:
#   `` make run_tests_${PROJECT_NAME}_nosetests_${dir}``
#   (where slashes in the ``dir`` are replaced with underscores)
#
# :param path: a relative or absolute directory to search for
#   nosetests in or a relative or absolute file containing tests
# :type path: string
# :param DEPENDENCIES: the targets which must be built before executing
#   the test
# :type DEPENDENCIES: list of strings
# :param TIMEOUT: the timeout for individual tests in seconds
#   (default: 60)
# :type TIMEOUT: integer
# :param WORKING_DIRECTORY: the working directory when executing the
#   tests
# :type WORKING_DIRECTORY: string
#
# @public
#
function(catkin_add_nosetests path)
  if(NOT NOSETESTS)
    message(STATUS "skipping nosetests(${path}) in project '${PROJECT_NAME}'")
    return()
  endif()

  cmake_parse_arguments(_nose "" "TIMEOUT;WORKING_DIRECTORY" "DEPENDENCIES" ${ARGN})
  if(NOT _nose_TIMEOUT)
    set(_nose_TIMEOUT 60)
  endif()
  if(NOT _nose_TIMEOUT GREATER 0)
    message(FATAL_ERROR "nosetests() TIMEOUT argument must be a valid number of seconds greater than zero")
  endif()

  # check that the directory exists
  set(_path_name _path_name-NOTFOUND)
  if(IS_ABSOLUTE ${path})
    set(_path_name ${path})
  else()
    find_file(_path_name ${path}
      PATHS ${CMAKE_CURRENT_SOURCE_DIR}
      NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
    if(NOT _path_name)
      message(FATAL_ERROR "Can't find nosetests path '${path}'")
    endif()
  endif()

  # check if coverage reports are being requested
  if("$ENV{CATKIN_TEST_COVERAGE}" STREQUAL "1")
    set(_covarg " --with-coverage")
  endif()

  set(output_path ${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME})
  string(REPLACE "/" "." output_file_name ${path})
  set(cmd "${CMAKE_COMMAND} -E make_directory ${output_path}")
  if(IS_DIRECTORY ${_path_name})
    set(tests "--where=${_path_name}")
  else()
    set(tests "${_path_name}")
  endif()
  set(cmd ${cmd} "${NOSETESTS} --process-timeout=${_nose_TIMEOUT} ${tests} --with-xunit --xunit-file=${output_path}/nosetests-${output_file_name}.xml${_covarg}")
  catkin_run_tests_target("nosetests" ${output_file_name} "nosetests-${output_file_name}.xml" COMMAND ${cmd} DEPENDENCIES ${_nose_DEPENDENCIES} WORKING_DIRECTORY ${_nose_WORKING_DIRECTORY})
endfunction()

function(add_nosetests)
  message(WARNING "add_nosetests() is deprecated, please rename the function call to catkin_add_nosetests()")
  catkin_add_nosetests(${ARGN})
endfunction()

find_program(NOSETESTS nosetests)
if(NOT nosetests_path)
  # retry with name including major version number
  find_program(NOSETESTS nosetests2)
endif()
if(NOT NOSETESTS)
  message(WARNING "nosetests not found, Python tests can not be run (try installing package 'python-nose')")
endif()
