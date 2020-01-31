function(catkin_generate_environment)
  set(SETUP_FILENAME "setup")

  # devel space
  set(SETUP_DIR ${CATKIN_DEVEL_PREFIX})

  # generate empty file to prevent searching for packages in binary dir
  # except if source space and build space are identical (which is the case for dry eclipse projects)
  if(NOT "${CMAKE_BINARY_DIR}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
    file(WRITE "${CMAKE_BINARY_DIR}/CATKIN_IGNORE" "")
  endif()

  # get multiarch name
  set(CATKIN_LIB_ENVIRONMENT_PATHS "'${CATKIN_GLOBAL_LIB_DESTINATION}'")
  set(CATKIN_PKGCONFIG_ENVIRONMENT_PATHS "os.path.join('${CATKIN_GLOBAL_LIB_DESTINATION}', 'pkgconfig')")
  if (UNIX AND NOT APPLE)
    # Two step looking for multiarch support: check for gcc -print-multiarch
    # and, if failed, try to run dpkg-architecture
    execute_process(COMMAND gcc -print-multiarch
                    OUTPUT_VARIABLE CATKIN_MULTIARCH
                    OUTPUT_STRIP_TRAILING_WHITESPACE
                    ERROR_QUIET
    )
    if ("${CATKIN_MULTIARCH}" STREQUAL "")
      execute_process(COMMAND dpkg-architecture -qDEB_HOST_MULTIARCH
                      OUTPUT_VARIABLE CATKIN_MULTIARCH
                      OUTPUT_STRIP_TRAILING_WHITESPACE
                      ERROR_QUIET
      )
    endif()
    if (NOT "${CATKIN_MULTIARCH}" STREQUAL "")
      set(CATKIN_LIB_ENVIRONMENT_PATHS
        "[${CATKIN_LIB_ENVIRONMENT_PATHS}, os.path.join('${CATKIN_GLOBAL_LIB_DESTINATION}', '${CATKIN_MULTIARCH}')]")
      set(CATKIN_PKGCONFIG_ENVIRONMENT_PATHS
        "[${CATKIN_PKGCONFIG_ENVIRONMENT_PATHS}, os.path.join('${CATKIN_GLOBAL_LIB_DESTINATION}', '${CATKIN_MULTIARCH}', 'pkgconfig')]")
    endif()
  endif()

  # generate Python setup util
  atomic_configure_file(${catkin_EXTRAS_DIR}/templates/_setup_util.py.in
    ${CATKIN_DEVEL_PREFIX}/_setup_util.py
    @ONLY)

  if(NOT WIN32)
    # non-windows
    # generate env
    atomic_configure_file(${catkin_EXTRAS_DIR}/templates/env.sh.in
      ${CATKIN_DEVEL_PREFIX}/env.sh
      @ONLY)
    # generate setup for various shells
    foreach(shell bash sh zsh)
      atomic_configure_file(${catkin_EXTRAS_DIR}/templates/setup.${shell}.in
        ${CATKIN_DEVEL_PREFIX}/setup.${shell}
        @ONLY)
      atomic_configure_file(${catkin_EXTRAS_DIR}/templates/local_setup.${shell}.in
        ${CATKIN_DEVEL_PREFIX}/local_setup.${shell}
        @ONLY)
    endforeach()

  else()
    # windows
    # generate env
    atomic_configure_file(${catkin_EXTRAS_DIR}/templates/env.bat.in
      ${CATKIN_DEVEL_PREFIX}/env.bat
      @ONLY)
    # generate setup
    atomic_configure_file(${catkin_EXTRAS_DIR}/templates/setup.bat.in
      ${CATKIN_DEVEL_PREFIX}/setup.bat
      @ONLY)
    atomic_configure_file(${catkin_EXTRAS_DIR}/templates/local_setup.bat.in
      ${CATKIN_DEVEL_PREFIX}/local_setup.bat
      @ONLY)
  endif()

  # generate rosinstall file referencing setup.sh
  atomic_configure_file(${catkin_EXTRAS_DIR}/templates/rosinstall.in
    ${CATKIN_DEVEL_PREFIX}/.rosinstall
    @ONLY)

  # installspace
  set(SETUP_DIR ${CMAKE_INSTALL_PREFIX})

  if(CATKIN_INSTALL_INTO_PREFIX_ROOT)
    # install empty workspace marker if it doesn't already exist
    install(CODE "
      if (NOT EXISTS \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}\")
        file(MAKE_DIRECTORY \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}\")
      endif()
      if (NOT EXISTS \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/.catkin\")
        file(WRITE \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/.catkin\" \"\")
      endif()")

    # generate and install Python setup util
    configure_file(${catkin_EXTRAS_DIR}/templates/_setup_util.py.in
      ${CMAKE_BINARY_DIR}/catkin_generated/installspace/_setup_util.py
      @ONLY)
    catkin_install_python(PROGRAMS
      ${CMAKE_BINARY_DIR}/catkin_generated/installspace/_setup_util.py
      DESTINATION ${CMAKE_INSTALL_PREFIX})
  endif()

  # initialize shell support per platform.
  set(CATKIN_ENV_SHELL sh)
  set(CATKIN_SETUP_SHELL bash sh zsh)
  if(WIN32)
    set(CATKIN_ENV_SHELL bat)
    set(CATKIN_SETUP_SHELL bat)
  endif()

  # generate and install env
  configure_file(${catkin_EXTRAS_DIR}/templates/env.${CATKIN_ENV_SHELL}.in
    ${CMAKE_BINARY_DIR}/catkin_generated/installspace/env.${CATKIN_ENV_SHELL}
    @ONLY)
  if(CATKIN_INSTALL_INTO_PREFIX_ROOT)
    install(PROGRAMS
      ${CMAKE_BINARY_DIR}/catkin_generated/installspace/env.${CATKIN_ENV_SHELL}
      DESTINATION ${CMAKE_INSTALL_PREFIX})
  endif()
  # generate and install setup for various shells
  foreach(shell ${CATKIN_SETUP_SHELL})
    configure_file(${catkin_EXTRAS_DIR}/templates/setup.${shell}.in
      ${CMAKE_BINARY_DIR}/catkin_generated/installspace/setup.${shell}
      @ONLY)
    configure_file(${catkin_EXTRAS_DIR}/templates/local_setup.${shell}.in
      ${CMAKE_BINARY_DIR}/catkin_generated/installspace/local_setup.${shell}
      @ONLY)
    if(CATKIN_INSTALL_INTO_PREFIX_ROOT)
      install(FILES
        ${CMAKE_BINARY_DIR}/catkin_generated/installspace/setup.${shell}
        ${CMAKE_BINARY_DIR}/catkin_generated/installspace/local_setup.${shell}
        DESTINATION ${CMAKE_INSTALL_PREFIX})
    endif()
  endforeach()

  # generate rosinstall file referencing setup.sh
  configure_file(${catkin_EXTRAS_DIR}/templates/rosinstall.in
    ${CMAKE_BINARY_DIR}/catkin_generated/installspace/.rosinstall
    @ONLY)
  if(CATKIN_INSTALL_INTO_PREFIX_ROOT)
    install(FILES
      ${CMAKE_BINARY_DIR}/catkin_generated/installspace/.rosinstall
      DESTINATION ${CMAKE_INSTALL_PREFIX})
  endif()
endfunction()
