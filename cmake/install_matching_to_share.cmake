function(install_matching_to_share GLOBEXPR)
  file(GLOB_RECURSE globbed
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    ${GLOBEXPR})

  foreach(file ${globbed})
    get_filename_component(pathonly ${file} PATH)
    get_filename_component(package ${pathonly} NAME)
    install(FILES ${file}
      DESTINATION share/${package}
      )
  endforeach()
endfunction()