function(atomic_configure_file input output)
  get_filename_component(atomic_file ${output} NAME)
  set(atomic_file "${CMAKE_BINARY_DIR}/${atomic_file}")
  configure_file("${input}" "${atomic_file}" ${ARGN})
  get_filename_component(output_path ${output} PATH)
  # sync multiple catkin cmake processes writing to that location
  file(LOCK "${output_path}" DIRECTORY GUARD FUNCTION)
  file(COPY "${atomic_file}" DESTINATION "${output_path}")
endfunction()
