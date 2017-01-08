include(${CMAKE_CURRENT_LIST_DIR}/../cmake/list_append_keys_from.cmake)

function(run_test_list_append_keys_from target input len)
  list(APPEND listname ${target})
  list(APPEND source ${input})

  list_append_keys_from(listname source DEBUG)
  
  list(LENGTH listname _len)
  if(NOT _len EQUAL len)
    message(SEND_ERROR "test failed, ${_len} != ${len}; '${listname}' <- '${target}' + '${input}'")
  endif()
endfunction()

run_test_list_append_keys_from("" "-DABC" 1)
run_test_list_append_keys_from("-DABC" "-DDEF" 2)
run_test_list_append_keys_from("-DABC" "-DABC1" 2)
  
run_test_list_append_keys_from("-DABC;" "-DABC;" 1)
run_test_list_append_keys_from("-DABC;" "-DABC=;" 1)
run_test_list_append_keys_from("-DABC;" "-DABC=1;" 1)
run_test_list_append_keys_from("-DABC;" "-DABC=2;" 1)

run_test_list_append_keys_from("-DABC=;" "-DABC;" 1)
run_test_list_append_keys_from("-DABC=;" "-DABC=;" 1)
run_test_list_append_keys_from("-DABC=;" "-DABC=1;" 1)
run_test_list_append_keys_from("-DABC=;" "-DABC=2;" 1)

run_test_list_append_keys_from("-DABC=1;" "-DABC;" 1)
run_test_list_append_keys_from("-DABC=1;" "-DABC=;" 1)
run_test_list_append_keys_from("-DABC=1;" "-DABC=1;" 1)
run_test_list_append_keys_from("-DABC=1;" "-DABC=2;" 1)

run_test_list_append_keys_from("-DABC=2;" "-DABC;" 1)
run_test_list_append_keys_from("-DABC=2;" "-DABC=;" 1)
run_test_list_append_keys_from("-DABC=2;" "-DABC=1;" 1)
run_test_list_append_keys_from("-DABC=2;" "-DABC=2;" 1)
