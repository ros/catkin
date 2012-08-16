function(install_matching_to_share GLOBEXPR)
  file(GLOB_RECURSE globbed
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    ${GLOBEXPR})

  foreach(file ${globbed})
    get_filename_component(pathonly ${file} PATH)
    set(package ${CATKIN_CURRENT_STACK})
    if(pathonly)
      get_filename_component(package ${pathonly} NAME)
    endif()
    install(FILES ${file}
      DESTINATION share/${CATKIN_CURRENT_STACK})
  endforeach()
endfunction()
