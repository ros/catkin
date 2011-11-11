function(stamp FILE)
  configure_file(${FILE}
    ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/stamps/${FILE}
    @ONLY)
endfunction()