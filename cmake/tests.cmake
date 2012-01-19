function(catkin_initialize_tests)
  if(NOT TARGET tests)
    add_custom_target(tests)
    message("TODO: implement add_roslaunch_check() in rostest-extras.cmake.")
  endif()
  if(NOT TARGET test)
    add_custom_target(test)
    add_dependencies(test tests)
  endif()
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
endfunction()

catkin_initialize_tests()

# Function to download data on the tests target
function(download_test_data _url _filename _md5)
  # Create a legal target name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname download_data_${_filename})
  add_custom_command(OUTPUT ${PROJECT_BINARY_DIR}/${_filename}
                     COMMAND ${catkin_EXTRAS_DIR}/test/download_checkmd5.py ${_url} ${PROJECT_BINARY_DIR}/${_filename} ${_md5}
                     VERBATIM)
  add_custom_target(${_testname} DEPENDS ${PROJECT_BINARY_DIR}/${_filename})
  add_dependencies(tests ${_testname})
endfunction()
