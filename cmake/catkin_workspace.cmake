function(catkin_workspace)
  em_expand(
    ${catkin_EXTRAS_DIR}/generate-context.in
    ${CMAKE_CURRENT_BINARY_DIR}/cmake/generate-context.py
    ${catkin_EXTRAS_DIR}/generate.cmake.em
    ${CMAKE_CURRENT_BINARY_DIR}/cmake/generate.cmake)
endfunction()

