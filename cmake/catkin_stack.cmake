function(catkin_stack)
  # Related to #78.
  if(CATKIN_CURRENT_STACK)
    message("Warning: catkin_stack(): in ${CMAKE_CURRENT_LIST_FILE}, CATKIN_CURRENT_STACK is already set (to: ${CATKIN_CURRENT_STACK}).  This should not be the case.")
  endif()

  stamp(${CMAKE_CURRENT_SOURCE_DIR}/stack.yaml)

  safe_execute_process(COMMAND ${PYTHON_EXECUTABLE}
    ${catkin_EXTRAS_DIR}/stack_get.py
    ${PROJECT_SOURCE_DIR}/stack.yaml
    ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/stack.cmake
    )

  include(${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/stack.cmake)

  assert(CATKIN_ENV)

endfunction()

