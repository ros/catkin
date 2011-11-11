function(catkin_workspace)
  assert(catkin_EXTRAS_DIR)
  assert_file_exists(${catkin_EXTRAS_DIR}/topologically_traverse.py.in "${catkin_EXTRAS_DIR}")
  assert_file_exists(${catkin_EXTRAS_DIR}/topologically_traverse.cmake.em "file nto found")
  em_expand(
    ${catkin_EXTRAS_DIR}/topologically_traverse.py.in
    ${CMAKE_CURRENT_BINARY_DIR}/topologically_traverse.py
    ${catkin_EXTRAS_DIR}/topologically_traverse.cmake.em
    ${CMAKE_CURRENT_BINARY_DIR}/topologically_traverse.cmake)
endfunction()

