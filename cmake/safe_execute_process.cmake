function(safe_execute_process arg1)

  set(msg ${arg1})
  foreach(arg ${ARGN})
    set(msg "${msg} ${arg}")
  endforeach()

  execute_process(${ARGV}
    RESULT_VARIABLE res)

  log(2 "Executing ${msg}")
  if (NOT res EQUAL 0)
    message(FATAL_ERROR "Error executing process with arguments ${msg}")
  endif()

endfunction()