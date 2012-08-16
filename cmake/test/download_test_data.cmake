# download data on the tests target
function(download_test_data _url _filename _md5)
  # create a legal target name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname download_data_${_filename})
  add_custom_command(OUTPUT ${PROJECT_BINARY_DIR}/${_filename}
    COMMAND ${catkin_EXTRAS_DIR}/test/download_checkmd5.py ${_url} ${PROJECT_BINARY_DIR}/${_filename} ${_md5}
    VERBATIM)
  add_custom_target(${_testname} DEPENDS ${PROJECT_BINARY_DIR}/${_filename})
  add_dependencies(tests ${_testname})
endfunction()
