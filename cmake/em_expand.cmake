macro(em_expand context_in context_out em_file_in file_out)
  assert_file_exists("${context_in}" "input file for context missing")
  assert_file_exists("${em_file_in}" "template file missing")
  debug_message(2 "configure_file(${context_in}, ${context_out})")
  configure_file(${context_in} ${context_out} @ONLY)
  assert_file_exists("${context_out}" "context file was not generated correctly")

  stamp(${em_file_in})

  debug_message(2 "Evaluate template '${em_file_in}' to '${file_out}' (with context from '${context_out}')")
  assert(EMPY_EXECUTABLE)
  set(command ${EMPY_EXECUTABLE})
  # prepend environment if set
  if(CATKIN_ENV)
    set(command ${CATKIN_ENV} ${command})
  endif()
  safe_execute_process(COMMAND
    ${command}
    --raw-errors
    -F ${context_out}
    -o ${file_out}
    ${em_file_in})
endmacro()
