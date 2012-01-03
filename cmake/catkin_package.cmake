function(catkin_package PKGNAME)

  stamp(${PROJECT_SOURCE_DIR}/stack.yaml)

  safe_execute_process(COMMAND
    ${catkin_EXTRAS_DIR}/stack_get.py
    ${PROJECT_SOURCE_DIR}/stack.yaml
    ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/stack.cmake
    )

  include(${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/stack.cmake)

  assert(CATKIN_ENV)

endfunction()

