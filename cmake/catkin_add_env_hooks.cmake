function(catkin_add_env_hooks ARG_ENV_HOOK)
  parse_arguments(ARG "DIRECTORY;SHELLS" "SKIP_INSTALL" ${ARGN})

  # create directory if necessary
  if(NOT IS_DIRECTORY ${catkin_BUILD_PREFIX}/etc/catkin/profile.d)
    file(MAKE_DIRECTORY ${catkin_BUILD_PREFIX}/etc/catkin/profile.d)
  endif()

  if(NOT ARG_DIRECTORY)
    set(ARG_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  endif()

  foreach(shell ${ARG_SHELLS})
    set(ENV_HOOK ${ARG_ENV_HOOK}.${shell})
    assert_file_exists(${ARG_DIRECTORY}/${ENV_HOOK}.in "User-supplied environment file '${ARG_DIRECTORY}/${ENV_HOOK}.in' missing")

    # generate environment hook for buildspace
    set(ENV_BUILDSPACE true)
    set(ENV_INSTALLSPACE false)
    configure_file(${ARG_DIRECTORY}/${ENV_HOOK}.in
      ${catkin_BUILD_PREFIX}/etc/catkin/profile.d/${ENV_HOOK})

    # generate and install environment hook for installspace
    set(ENV_BUILDSPACE false)
    set(ENV_INSTALLSPACE true)
    configure_file(${ARG_DIRECTORY}/${ENV_HOOK}.in
      ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${ENV_HOOK})
    if(NOT ${ARG_SKIP_INSTALL})
      install(FILES ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${ENV_HOOK}
        DESTINATION etc/catkin/profile.d)
    endif()
  endforeach()
endfunction()
