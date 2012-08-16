function(stamp path)
  get_filename_component(filename ${path} NAME)
  configure_file(${path}
    ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/stamps/${PROJECT_NAME}/${filename}
    @ONLY)
endfunction()
