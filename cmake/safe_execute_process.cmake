function(safe_execute_process cmd_keyword arg1)
  set(cmd ${arg1})
  foreach(arg ${ARGN})
    set(cmd "${cmd} ${arg}")
  endforeach()

  debug_message(2 "execute_process(${cmd})")
  execute_process(${ARGV} RESULT_VARIABLE res)

  if(NOT res EQUAL 0)
    message(FATAL_ERROR "execute_process(${cmd}) returned error code ${res}")
  endif()
endfunction()
