function(catkin_package)
  message("VESTIGIAL: catkin_package unused in ${CMAKE_CURRENT_SOURCE_DIR}, change to catkin_stack.")
  catkin_stack()
endfunction()

function(catkin_stack)

  stamp(${CMAKE_CURRENT_SOURCE_DIR}/stack.yaml)

  safe_execute_process(COMMAND
    ${catkin_EXTRAS_DIR}/stack_get.py
    ${PROJECT_SOURCE_DIR}/stack.yaml
    ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/stack.cmake
    )

  include(${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/stack.cmake)

  assert(CATKIN_ENV)

endfunction()

