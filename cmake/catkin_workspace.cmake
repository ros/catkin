function(catkin_workspace)
  assert(catkin_EXTRAS_DIR)
  assert_file_exists(${catkin_EXTRAS_DIR}/topologically_traverse.py.in "${catkin_EXTRAS_DIR}")
  assert_file_exists(${catkin_EXTRAS_DIR}/topologically_traverse.cmake.em "file nto found")
  include_directories(${CMAKE_BINARY_DIR}/gen/cpp)

  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

  em_expand(
    ${catkin_EXTRAS_DIR}/topologically_traverse.py.in
    ${CMAKE_CURRENT_BINARY_DIR}/topologically_traverse.py
    ${catkin_EXTRAS_DIR}/topologically_traverse.cmake.em
    ${CMAKE_CURRENT_BINARY_DIR}/topologically_traverse.cmake)

  foreach(shfile setup.sh setup.zsh setup.bash env.sh)
    configure_file(${catkin_EXTRAS_DIR}/${shfile}.in ${CMAKE_BINARY_DIR}/${shfile} 
      @ONLY)
  endforeach()

endfunction()

