find_program(NOSETESTS nosetests)
if(NOT nosetests_path)
  # retry with name including major version number
  find_program(NOSETESTS nosetests2)
endif()
if(NOT NOSETESTS)
  message(WARNING "nosetests not found, Python tests can not be run (try installing package 'python-nose')")
endif()

function(add_nosetests dir)
  if(NOT NOSETESTS)
    message(STATUS "skipping nosetests '${dir}' in project '${PROJECT_NAME}'")
    return()
  endif()

  parse_arguments(_nose "WORKING_DIRECTORY" "" ${ARGN})
  if(_nose_WORKING_DIRECTORY)
    set(_chdir_prefix "bash -c \"cd ${_nose_WORKING_DIRECTORY} && ")
    set(_chdir_suffix "\"")
  endif()

  # check that the directory exists
  set(_dir_name _dir_name-NOTFOUND)
  if(IS_ABSOLUTE ${dir})
    set(_dir_name ${dir})
  else()
    find_file(_dir_name ${dir}
              PATHS ${CMAKE_CURRENT_SOURCE_DIR}
              NO_DEFAULT_PATH
              NO_CMAKE_FIND_ROOT_PATH)  # for cross-compilation.  thanks jeremy.
    if(NOT _dir_name)
      message(FATAL_ERROR "Can't find nosetests dir \"${dir}\"")
    endif()
  endif()

  # check if coverage reports are being requested
  if("$ENV{ROS_TEST_COVERAGE}" STREQUAL "1")
    set(_covarg "--with-coverage")
  else()
    set(_covarg)
  endif()

  set(output_dir_name ${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME})
  append_test_to_cache(catkin-tests "${CMAKE_COMMAND} -E make_directory ${output_dir_name}")
  string(REPLACE "/" "." output_file_name ${dir})
  append_test_to_cache(catkin-tests "${_chdir_prefix}${NOSETESTS} --where=${_dir_name} --with-xunit --xunit-file=${output_dir_name}/${output_file_name}.xml ${_covarg}${_chdir_suffix}")
  append_test_to_cache(catkin-tests "${CHECK_TEST_RAN_EXE} ${output_dir_name}/${output_file_name}.xml")
endfunction()
