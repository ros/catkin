function(shell arg1)

  execute_process(COMMAND ${arg1} ${ARGN}
    RESULT_VARIABLE res
    OUTPUT_VARIABLE out
    ERROR_VARIABLE out)

  if (res EQUAL 0)
    log(2 "execute_command ${arg1} OK: ${out}")
  else()
    set(msg "${arg1}")
    foreach(arg ${ARGN})
      set(msg "${msg} ${arg}")
    endforeach()
    message(FATAL_ERROR "execute_command: ${msg}\n******FAILED WITH ERROR:******\n${out}")
  endif()

endfunction()