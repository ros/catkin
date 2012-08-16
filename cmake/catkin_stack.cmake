function(catkin_stack)
  debug_message(10 "catkin_stack() called in file ${CMAKE_CURRENT_LIST_FILE}")

  # verify that no arguments are passed
  if(ARGN)
    message(FATAL_ERROR "catkin_stack() does not support arguments")
  endif()

  # ensure that function is not called multiple times per stack
  if(_CATKIN_CURRENT_STACK_INITIALIZED_)
    message(FATAL_ERROR "catkin_stack(): in '${CMAKE_CURRENT_LIST_FILE}', CATKIN_CURRENT_STACK is already set (to: ${CATKIN_CURRENT_STACK}).  This should not be the case.")
  endif()
  set(_CATKIN_CURRENT_STACK_INITIALIZED_)

  # stamp and parse stack.xml once
  if(NOT CATKIN_CURRENT_STACK)
    stamp(${CMAKE_CURRENT_SOURCE_DIR}/stack.xml)
    safe_execute_process(COMMAND ${PYTHON_EXECUTABLE}
      ${catkin_EXTRAS_DIR}/parse_stack_xml.py
      ${CMAKE_CURRENT_SOURCE_DIR}/stack.xml
      ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/stack.cmake)
  endif()
  # load extracted variable into cmake
  include(${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/stack.cmake)

  # install stack.xml
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/stack.xml
    DESTINATION share/${CATKIN_CURRENT_STACK})
endfunction()
