function(catkin_add_env_hooks ARG_PATH)
  parse_arguments(ARG "SHELLS" "" ${ARGN})

  # create directory if necessary
  if(NOT IS_DIRECTORY ${catkin_BUILD_PREFIX}/etc/catkin/profile.d)
    file(MAKE_DIRECTORY ${catkin_BUILD_PREFIX}/etc/catkin/profile.d)
  endif()

  foreach(shell ${ARG_SHELLS})
    _catkin_add_env_hooks(${ARG_PATH}.${shell})
  endforeach()
endfunction()

function(_catkin_add_env_hooks ENV_HOOK)
  assert_file_exists(${CMAKE_CURRENT_SOURCE_DIR}/${ENV_HOOK}.in "User-supplied environment file '${CMAKE_CURRENT_SOURCE_DIR}/${ENV_HOOK}.in' missing")

  # generate environment hook for buildspace
  set(ENV_BUILDSPACE true)
  set(ENV_INSTALLSPACE false)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${ENV_HOOK}.in
    ${catkin_BUILD_PREFIX}/etc/catkin/profile.d/${ENV_HOOK})

  # generate and install environment hook for installspace
  set(ENV_BUILDSPACE false)
  set(ENV_INSTALLSPACE true)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${ENV_HOOK}.in
    ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${ENV_HOOK})
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${ENV_HOOK}
    DESTINATION etc/catkin/profile.d)
endfunction()
