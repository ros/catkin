function(catkin_generate_environment)
  set(CATKIN_WORKSPACES $ENV{CATKIN_WORKSPACES})

  # buildspace
  set(SETUP_DIR ${catkin_BUILD_PREFIX})
  if(NOT MSVC)
    # non-windows
    # generate env
    configure_file(${catkin_EXTRAS_DIR}/templates/env.sh.in
      ${catkin_BUILD_PREFIX}/env.sh)
    # generate setup for various shells
    set(CURRENT_WORKSPACE ${catkin_BUILD_PREFIX}:${CMAKE_SOURCE_DIR})
    em_expand(${catkin_EXTRAS_DIR}/templates/setup.context.py.in
      ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/setup.buildspace.context.py
      ${catkin_EXTRAS_DIR}/em/setup.sh.em
      ${catkin_BUILD_PREFIX}/setup.sh)
    foreach(shell bash zsh)
      configure_file(${catkin_EXTRAS_DIR}/templates/setup.${shell}.in
        ${catkin_BUILD_PREFIX}/setup.${shell})
    endforeach()

  else()
    # windows
    # generate env
    configure_file(${catkin_EXTRAS_DIR}/templates/env.bat.in
      ${catkin_BUILD_PREFIX}/env.bat)
    # generate setup
    em_expand(${catkin_EXTRAS_DIR}/templates/setup.context.py.in
      ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/setup.buildspace.context.py
      ${catkin_EXTRAS_DIR}/em/setup.bat.em
      ${catkin_BUILD_PREFIX}/setup.bat)
  endif()

  # installspace
  set(SETUP_DIR ${CMAKE_INSTALL_PREFIX})
  if(NOT MSVC)
    # non-windows
    # generate and install env
    configure_file(${catkin_EXTRAS_DIR}/templates/env.sh.in
      ${CMAKE_CURRENT_BINARY_DIR}/installspace/env.sh)
    install(PROGRAMS
      ${CMAKE_CURRENT_BINARY_DIR}/installspace/env.sh
      DESTINATION ${CMAKE_INSTALL_PREFIX})
    # generate and install setup for various shells
    set(CURRENT_WORKSPACE ${CMAKE_INSTALL_PREFIX})
    em_expand(${catkin_EXTRAS_DIR}/templates/setup.context.py.in
      ${CMAKE_CURRENT_BINARY_DIR}/setup.installspace.context.py
      ${catkin_EXTRAS_DIR}/em/setup.sh.em
      ${CMAKE_CURRENT_BINARY_DIR}/installspace/setup.sh)
    install(FILES
      ${CMAKE_CURRENT_BINARY_DIR}/installspace/setup.sh
      DESTINATION ${CMAKE_INSTALL_PREFIX})
    foreach(shell bash zsh)
      configure_file(${catkin_EXTRAS_DIR}/templates/setup.${shell}.in
        ${CMAKE_CURRENT_BINARY_DIR}/installspace/setup.${shell})
      install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/installspace/setup.${shell}
        DESTINATION ${CMAKE_INSTALL_PREFIX})
    endforeach()

  else()
    # windows
    # generate and install env
    configure_file(${catkin_EXTRAS_DIR}/templates/env.bat.in
      ${CMAKE_CURRENT_BINARY_DIR}/installspace/env.bat)
    install(PROGRAMS
      ${CMAKE_CURRENT_BINARY_DIR}/installspace/env.bat
      DESTINATION ${CMAKE_INSTALL_PREFIX})
    # generate and install setup
    em_expand(${catkin_EXTRAS_DIR}/templates/setup.context.py.in
      ${CMAKE_CURRENT_BINARY_DIR}/setup.installspace.context.py
      ${catkin_EXTRAS_DIR}/em/setup.bat.em
      ${CMAKE_CURRENT_BINARY_DIR}/installspace/setup.bat)
    install(FILES
      ${CMAKE_CURRENT_BINARY_DIR}/installspace/setup.bat
      DESTINATION ${CMAKE_INSTALL_PREFIX})
  endif()

  # XXX what is .rosinstall needed for?
  #if(catkin_SOURCE_DIR)
  #  message(STATUS "foo ${CMAKE_CURRENT_BINARY_DIR}")
  #  configure_file(${catkin_EXTRAS_DIR}/templates/rosinstall.installable.in
  #    ${CMAKE_CURRENT_BINARY_DIR}/installspace/.rosinstall)
  #  install(FILES
  #    ${CMAKE_CURRENT_BINARY_DIR}/installspace/.rosinstall
  #    DESTINATION ${CMAKE_INSTALL_PREFIX})
  #endif()
endfunction()
