function(catkin_stack)
  check_unused_arguments("catkin_stack" "${ARGN}")

  # verify that project() has been called before
  if(NOT PROJECT_NAME)
    message(FATAL_ERROR "catkin_stack() PROJECT_NAME is not set. You must call project() before you can call catkin_stack().")
  endif()
  if(PROJECT_NAME STREQUAL Project)
message(FATAL_ERROR "catkin_stack() PROJECT_NAME is not set. The variable is set to 'Project', which is not an allowed project name. You must call project() before you can call catkin_stack().")
  endif()

  # Related to #78.
  if(CATKIN_CURRENT_STACK_INITIALIZED)
    message(WARNING "catkin_stack(): in ${CMAKE_CURRENT_LIST_FILE}, CATKIN_CURRENT_STACK is already set (to: ${CATKIN_CURRENT_STACK}).  This should not be the case.")
  else()
  
    if(NOT CATKIN_CURRENT_STACK)
      stamp(${CMAKE_CURRENT_SOURCE_DIR}/stack.xml)
      
      safe_execute_process(COMMAND ${PYTHON_EXECUTABLE}
	${catkin_EXTRAS_DIR}/stack_get.py
	${PROJECT_SOURCE_DIR}/stack.xml
	${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/stack.cmake
	)
    endif()
    include(${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/stack.cmake)
    set(CATKIN_CURRENT_STACK_INITIALIZED True)

  endif()

  message(STATUS "Building stack ${CATKIN_CURRENT_STACK}")
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/stack.xml
          DESTINATION share/${CATKIN_CURRENT_STACK}
  )

  assert(CATKIN_ENV)

endfunction()

