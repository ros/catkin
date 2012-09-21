function(catkin_generate_environment)
  # buildspace
  set(SETUP_DIR ${CATKIN_BUILD_PREFIX})

  # generate empty file to prevent searching for packages in binary dir
  file(WRITE "${CMAKE_BINARY_DIR}/CATKIN_NO_SUBDIRS" "")

  # create workspace marker
  set(sourcespaces "${CMAKE_SOURCE_DIR}")
  if(EXISTS "${CATKIN_BUILD_PREFIX}/.CATKIN_WORKSPACE")
    # prepend to existing list of sourcespaces
    file(READ "${CATKIN_BUILD_PREFIX}/.CATKIN_WORKSPACE" existing_sourcespaces)
    list(FIND existing_sourcespaces "${CMAKE_SOURCE_DIR}" _index)
    if(_index EQUAL -1)
      list(INSERT existing_sourcespaces 0 ${CMAKE_SOURCE_DIR})
    endif()
    set(sourcespaces ${existing_sourcespaces})
  endif()
  file(WRITE "${CATKIN_BUILD_PREFIX}/.CATKIN_WORKSPACE" "${sourcespaces}")

  # generate relay-script for setup.py
  set(PYTHON_SCRIPT ${catkin_EXTRAS_DIR}/templates/setup.py)
  configure_file(${catkin_EXTRAS_DIR}/templates/script.py.in
    ${CATKIN_BUILD_PREFIX}/setup.py
    @ONLY)

  if(NOT MSVC)
    # non-windows
    # generate env
    configure_file(${catkin_EXTRAS_DIR}/templates/env.sh.in
      ${CATKIN_BUILD_PREFIX}/env.sh
      @ONLY)
    # generate setup for various shells
    em_expand(${catkin_EXTRAS_DIR}/templates/setup.context.py.in
      ${CMAKE_BINARY_DIR}/catkin_generated/setup.buildspace.context.py
      ${catkin_EXTRAS_DIR}/em/setup.sh.em
      ${CATKIN_BUILD_PREFIX}/setup.sh)
    foreach(shell bash zsh)
      configure_file(${catkin_EXTRAS_DIR}/templates/setup.${shell}.in
        ${CATKIN_BUILD_PREFIX}/setup.${shell}
        @ONLY)
    endforeach()

  else()
    # windows
    # generate env
    configure_file(${catkin_EXTRAS_DIR}/templates/env.bat.in
      ${CATKIN_BUILD_PREFIX}/env.bat
      @ONLY)
    # generate setup
    em_expand(${catkin_EXTRAS_DIR}/templates/setup.context.py.in
      ${CMAKE_BINARY_DIR}/catkin_generated/setup.buildspace.context.py
      ${catkin_EXTRAS_DIR}/em/setup.bat.em
      ${CATKIN_BUILD_PREFIX}/setup.bat)
  endif()

  # generate rosinstall file referencing setup.sh
  configure_file(${catkin_EXTRAS_DIR}/templates/rosinstall.in
    ${CATKIN_BUILD_PREFIX}/.rosinstall
    @ONLY)

  # installspace
  set(SETUP_DIR ${CMAKE_INSTALL_PREFIX})

  if(NOT CATKIN_BUILD_BINARY_PACKAGE OR "${PROJECT_NAME}" STREQUAL "catkin")
    # generate and install workspace marker
    file(WRITE ${CMAKE_BINARY_DIR}/catkin_generated/installspace/.CATKIN_WORKSPACE "")
    install(FILES
      ${CMAKE_BINARY_DIR}/catkin_generated/installspace/.CATKIN_WORKSPACE
      DESTINATION ${CMAKE_INSTALL_PREFIX})
    # install setup.py
    install(PROGRAMS
      ${catkin_EXTRAS_DIR}/templates/setup.py
      DESTINATION ${CMAKE_INSTALL_PREFIX})
  endif()

  if(NOT MSVC)
    # non-windows
    # generate and install env
    configure_file(${catkin_EXTRAS_DIR}/templates/env.sh.in
      ${CMAKE_BINARY_DIR}/catkin_generated/installspace/env.sh
      @ONLY)
    if(NOT CATKIN_BUILD_BINARY_PACKAGE OR "${PROJECT_NAME}" STREQUAL "catkin")
      install(PROGRAMS
        ${CMAKE_BINARY_DIR}/catkin_generated/installspace/env.sh
        DESTINATION ${CMAKE_INSTALL_PREFIX})
    endif()
    # generate and install setup for various shells
    em_expand(${catkin_EXTRAS_DIR}/templates/setup.context.py.in
      ${CMAKE_BINARY_DIR}/catkin_generated/setup.installspace.context.py
      ${catkin_EXTRAS_DIR}/em/setup.sh.em
      ${CMAKE_BINARY_DIR}/catkin_generated/installspace/setup.sh)
    if(NOT CATKIN_BUILD_BINARY_PACKAGE OR "${PROJECT_NAME}" STREQUAL "catkin")
      install(FILES
        ${CMAKE_BINARY_DIR}/catkin_generated/installspace/setup.sh
        DESTINATION ${CMAKE_INSTALL_PREFIX})
    endif()
    foreach(shell bash zsh)
      configure_file(${catkin_EXTRAS_DIR}/templates/setup.${shell}.in
        ${CMAKE_BINARY_DIR}/catkin_generated/installspace/setup.${shell}
        @ONLY)
      if(NOT CATKIN_BUILD_BINARY_PACKAGE OR "${PROJECT_NAME}" STREQUAL "catkin")
        install(FILES
          ${CMAKE_BINARY_DIR}/catkin_generated/installspace/setup.${shell}
          DESTINATION ${CMAKE_INSTALL_PREFIX})
      endif()
    endforeach()

  else()
    # windows
    # generate and install env
    configure_file(${catkin_EXTRAS_DIR}/templates/env.bat.in
      ${CMAKE_BINARY_DIR}/catkin_generated/installspace/env.bat
      @ONLY)
    install(PROGRAMS
      ${CMAKE_BINARY_DIR}/catkin_generated/installspace/env.bat
      DESTINATION ${CMAKE_INSTALL_PREFIX})
    # generate and install setup
    em_expand(${catkin_EXTRAS_DIR}/templates/setup.context.py.in
      ${CMAKE_BINARY_DIR}/catkin_generated/setup.installspace.context.py
      ${catkin_EXTRAS_DIR}/em/setup.bat.em
      ${CMAKE_BINARY_DIR}/catkin_generated/installspace/setup.bat)
    install(FILES
      ${CMAKE_BINARY_DIR}/catkin_generated/installspace/setup.bat
      DESTINATION ${CMAKE_INSTALL_PREFIX})
  endif()

  # generate rosinstall file referencing setup.sh
  configure_file(${catkin_EXTRAS_DIR}/templates/rosinstall.in
    ${CMAKE_BINARY_DIR}/catkin_generated/installspace/.rosinstall
    @ONLY)
  if(NOT CATKIN_BUILD_BINARY_PACKAGE OR "${PROJECT_NAME}" STREQUAL "catkin")
    install(FILES
      ${CMAKE_BINARY_DIR}/catkin_generated/installspace/.rosinstall
      DESTINATION ${CMAKE_INSTALL_PREFIX})
  endif()
endfunction()
