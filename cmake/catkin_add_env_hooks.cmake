if (NOT IS_DIRECTORY ${CMAKE_BINARY_DIR}/etc/catkin/profile.d)
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/etc/catkin/profile.d)
endif()

function(catkin_add_env_hooks_impl BUILDSPACE INSTALLSPACE)

  assert_file_exists(${CMAKE_CURRENT_SOURCE_DIR}/${BUILDSPACE}.in
    "User-supplied environment file (buildspace) missing")

  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${BUILDSPACE}.in
    ${CMAKE_BINARY_DIR}/etc/catkin/profile.d/${BUILDSPACE}
    )

  assert_file_exists(${CMAKE_CURRENT_SOURCE_DIR}/${INSTALLSPACE}.in
    "User-supplied environment file (installable) missing")

  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${INSTALLSPACE}.in
    ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${INSTALLSPACE}
    )

  install(FILES ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${INSTALLSPACE}
    DESTINATION
    etc/catkin/profile.d)

endfunction()

function(catkin_add_env_hooks ARG_PATH)

  parse_arguments(ARG
    "SHELLS" ""
    ${ARGN})

  foreach(shell ${ARG_SHELLS})
    catkin_add_env_hooks_impl(
      ${ARG_PATH}.buildspace.${shell}
      ${ARG_PATH}.${shell}
      )
  endforeach()

endfunction()


function(catkin_generic_hooks)
  if(CMAKE_HOST_WIN32)
    set(CATKIN_PATH_SEPARATOR ";")
  else()
    set(CATKIN_PATH_SEPARATOR ":")
  endif()
  foreach(path ${CATKIN_ROSDEPS_PATH})
	file(TO_NATIVE_PATH ${path}/bin _binpath)
	file(TO_NATIVE_PATH ${path}/lib _libpath)
	set(CATKIN_ROSDEPS_BINARY_PATH ${CATKIN_ROSDEPS_BINARY_PATH}${CATKIN_PATH_SEPARATOR}${_binpath}${CATKIN_PATH_SEPARATOR}${_libpath})
  endforeach()
  if(MSVC)
    # TODO: windows .bat versions, currently only for buildspace
    configure_file(${catkin_EXTRAS_DIR}/templates/setup.bat.buildspace.in
      ${CMAKE_BINARY_DIR}/setup.bat)
    configure_file(${catkin_EXTRAS_DIR}/templates/env.bat.buildspace.in
      ${CMAKE_BINARY_DIR}/env.bat)
    configure_file(${catkin_EXTRAS_DIR}/templates/setup.bat.installable.in
      ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/setup.bat)
    configure_file(${catkin_EXTRAS_DIR}/templates/env.bat.installable.in
      ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/env.bat)
    install(FILES
      ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/setup.bat
              DESTINATION ${CMAKE_INSTALL_PREFIX}
    )
    install(PROGRAMS
      ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/env.bat
      DESTINATION ${CMAKE_INSTALL_PREFIX}
    )
  else(MSVC) # Not msvc
    foreach(shell sh bash zsh)
      configure_file(${catkin_EXTRAS_DIR}/templates/setup.${shell}.buildspace.in
        ${CMAKE_BINARY_DIR}/setup.${shell})
      configure_file(${catkin_EXTRAS_DIR}/templates/setup.${shell}.installable.in
        ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/setup.${shell})
      if(catkin_SOURCE_DIR) #TODO FIXME these should only be installed if this is catkin?
        install(FILES
          ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/setup.${shell}
          DESTINATION ${CMAKE_INSTALL_PREFIX}
          )
      endif()
    endforeach()
    configure_file(${catkin_EXTRAS_DIR}/templates/env.sh.installable.in
      ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/env.sh)
    configure_file(${catkin_EXTRAS_DIR}/templates/env.sh.buildspace.in
      ${CMAKE_BINARY_DIR}/env.sh)
    if(catkin_SOURCE_DIR) #TODO FIXME these should only be installed if this is catkin?
      install(PROGRAMS
        ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/env.sh
        DESTINATION ${CMAKE_INSTALL_PREFIX}
      )
    endif()
  endif(MSVC)
  
  configure_file(${catkin_EXTRAS_DIR}/templates/rosinstall.installable.in
    ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/.rosinstall)

  if(catkin_SOURCE_DIR) #TODO FIXME these should only be installed if this is catkin?
    install(FILES
      ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/.rosinstall
      DESTINATION ${CMAKE_INSTALL_PREFIX})
  endif()

endfunction()
