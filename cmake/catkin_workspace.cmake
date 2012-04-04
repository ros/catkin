function(catkin_workspace)
  if (NOT IS_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
  endif()

  assert(catkin_EXTRAS_DIR)
  assert_file_exists(${catkin_EXTRAS_DIR}/templates/topologically_traverse.py.in "${catkin_EXTRAS_DIR}")
  assert_file_exists(${catkin_EXTRAS_DIR}/em/topologically_traverse.cmake.em "file not found")
  include_directories(${CMAKE_BINARY_DIR}/gen/cpp)

  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

  # This is convenient when the platform has no 'system' path (e.g. windows).
  set(CATKIN_ROSDEPS_PATH "" CACHE STRING "Semi-colon separated list of root directories to search for rosdeps.") 
  foreach(path ${CATKIN_ROSDEPS_PATH})
    message(STATUS "Rosdep root search path added: ${path}")
    # Help cmake's find_file and find_library
    list(APPEND CMAKE_PREFIX_PATH ${path})
    # CMAKE_PREFIX_PATH doesn't add it to the compile search path for includes.
    include_directories(${path}/include)
  endforeach()

  # libraries.cmake
  configure_shared_library_build_settings()

  em_expand(
    ${catkin_EXTRAS_DIR}/templates/topologically_traverse.py.in
    ${CMAKE_CURRENT_BINARY_DIR}/topologically_traverse.py
    ${catkin_EXTRAS_DIR}/em/topologically_traverse.cmake.em
    ${CMAKE_CURRENT_BINARY_DIR}/topologically_traverse.cmake
    )

endfunction()

