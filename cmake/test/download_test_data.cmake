_generate_function_if_testing_is_disabled("download_test_data")

# download data on the tests target
function(download_test_data _url _filename _md5)
  _warn_if_skip_testing("download_test_data")

  # create a legal target name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname download_data_${_filename})
  message(WARNING "download_test_data() is deprecated, please use catkin_download_test_data() instead.\nUse the following signature:\ncatkin_download_test_data(${_testname} ${_url} FILENAME ${_filename} MD5 ${_md5})")
  catkin_download_test_data(${_testname} ${_url} FILENAME ${_filename} MD5 ${_md5})
endfunction()
