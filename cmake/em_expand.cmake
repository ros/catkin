
macro(em_expand CONTEXT_FILE_IN CONTEXT_FILE_OUT EM_FILE_IN CMAKE_FILE_OUT)
  if (NOT EXISTS ${CONTEXT_FILE_IN})
    message(FATAL_ERROR "Attempt to expand nonexistent file ${CONTEXT_FILE_IN}")
  endif()
  configure_file(${CONTEXT_FILE_IN} ${CONTEXT_FILE_OUT})
  log(2 "Expanding ${EM_FILE_IN} from ${CONTEXT_FILE_OUT}")

  assert_file_exists("${CONTEXT_FILE_OUT}" "CONTEXT_FILE_OUT")
  assert_file_exists("${CATKIN_CONTEXT_FILE}" "CATKIN_CONTEXT_FILE")
  assert_file_exists(${catkin_EXTRAS_DIR}/empy_util.py "CATKIN_CONTEXT_FILE")

  configure_file(${EM_FILE_IN} ${CMAKE_CURRENT_BINARY_DIR}${EM_FILE_IN}.stamp @ONLY)

  safe_execute_process(COMMAND
    ${EMPY_EXECUTABLE}
    --raw-errors
    -F ${catkin_EXTRAS_DIR}/empy_util.py
    -F ${CONTEXT_FILE_OUT}
    -F ${CATKIN_CONTEXT_FILE}
    -o ${CMAKE_FILE_OUT}
    ${EM_FILE_IN}
    )
  log(2 STATUS "*** including ${CMAKE_FILE_OUT}")
  include(${CMAKE_FILE_OUT})
endmacro()
