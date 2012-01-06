if (NOT IS_DIRECTORY ${CMAKE_BINARY_DIR}/etc/catkin/env.d)
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/etc/catkin/env.d)
endif()

function(catkin_add_env_hooks BUILDSPACE INSTALLSPACE)

  configure_file(${BUILDSPACE}.in ${CMAKE_BINARY_DIR}/etc/catkin/env.d/${BUILDSPACE})
  configure_file(${INSTALLSPACE}.in ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${INSTALLSPACE})

  install(FILES ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${INSTALLSPACE}
    DESTINATION
    etc/catkin/env.d)

endfunction()

