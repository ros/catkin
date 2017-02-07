include(${CMAKE_CURRENT_LIST_DIR}/../cmake/list_insert_options.cmake)

function(run_test_list_insert_options target input len)
  list(APPEND dest ${target})
  list(APPEND source ${input})

  list_insert_options(dest ${source} CONTEXT ${ARGV3} LEVEL STATUS)

  list(LENGTH dest _len)
  if(NOT _len EQUAL len)
    message(SEND_ERROR "test failed, length ${_len} should be ${len}; '${dest}' <- '${target}' + '${input}'")
  endif()
endfunction()

run_test_list_insert_options("" "-DABC" 1)
run_test_list_insert_options("-DABC" "-DDEF" 2)
run_test_list_insert_options("-DABC" "-DABC1" 2)

run_test_list_insert_options("-DABC;" "-DABC;" 1)
run_test_list_insert_options("-DABC;" "-DABC=;" 1)
run_test_list_insert_options("-DABC;" "-DABC=1;" 1  "my_context")
run_test_list_insert_options("-DABC;" "-DABC=2;" 1  "my_context")

run_test_list_insert_options("-DABC=;" "-DABC;" 1)
run_test_list_insert_options("-DABC=;" "-DABC=;" 1)
run_test_list_insert_options("-DABC=;" "-DABC=1;" 1)
run_test_list_insert_options("-DABC=;" "-DABC=2;" 1)

run_test_list_insert_options("-DABC=1;" "-DABC;" 1)
run_test_list_insert_options("-DABC=1;" "-DABC=;" 1)
run_test_list_insert_options("-DABC=1;" "-DABC=1;" 1)
run_test_list_insert_options("-DABC=1;" "-DABC=2;" 1)

run_test_list_insert_options("-DABC=2;" "-DABC;" 1)
run_test_list_insert_options("-DABC=2;" "-DABC=;" 1)
run_test_list_insert_options("-DABC=2;" "-DABC=1;" 1)
run_test_list_insert_options("-DABC=2;" "-DABC=2;" 1)
