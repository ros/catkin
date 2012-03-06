function(stamp FILE)
  get_filename_component(FILE_ONLY ${FILE} NAME)
  configure_file(${FILE}
    ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/stamps/${PROJECT_NAME}/${FILE_ONLY}
    @ONLY)
endfunction()