function(enable_python pkg_name)
  set(PACKAGE_NAME ${pkg_name})
  if(${pkg_name}_PYTHONPATH)
    set(PACKAGE_PYTHONPATH ${CMAKE_CURRENT_SOURCE_DIR}/${${pkg_name}_PYTHONPATH})
  else()
    set(PACKAGE_PYTHONPATH ${CMAKE_CURRENT_SOURCE_DIR}/src/${pkg_name})
  endif()

  configure_file(${catkin_EXTRAS_DIR}/templates/__init__.py.in
    ${CMAKE_BINARY_DIR}/gen/py/${pkg_name}/__init__.py
    @ONLY)

  if(EXISTS ${${pkg_name}_SOURCE_DIR}/setup.py)
    assert(INSTALLED_PYTHONPATH)
    set(INSTALL_CMD_WORKING_DIRECTORY ${${pkg_name}_SOURCE_DIR})
    set(INSTALL_SCRIPT
      ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/python_distutils_install.sh)

    configure_file(${catkin_EXTRAS_DIR}/templates/python_distutils_install.sh.in
      ${INSTALL_SCRIPT}
      @ONLY)

    configure_file(${catkin_EXTRAS_DIR}/templates/safe_execute_install.cmake.in
      ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/safe_execute_install.cmake)

    install(CODE "message(COMMAND = ${CMD})")
    install(SCRIPT ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/safe_execute_install.cmake)

    stamp(${${pkg_name}_SOURCE_DIR}/setup.py)

    set(CMD
      ${CATKIN_ENV} ${PYTHON_EXECUTABLE}
      ${catkin_EXTRAS_DIR}/interrogate_setup_dot_py.py
      ${pkg_name}
      ${${pkg_name}_SOURCE_DIR}/setup.py
      ${${pkg_name}_BINARY_DIR}/setup_py_interrogation.cmake
      )

    # message("IN ${pkg_name}:  ${CMD}")
    execute_process(COMMAND
      ${CMD}
      RESULT_VARIABLE RES
      )
    if (RES)
      message(FATAL_ERROR "Attempt to interrogate setup.py of project ${pkg_name} returned ${RES}")
    endif()
    include(${${pkg_name}_BINARY_DIR}/setup_py_interrogation.cmake)

    foreach(pkg ${${pkg_name}_PACKAGES})
      get_filename_component(name ${pkg} NAME)
      execute_process(COMMAND /bin/ln -sf
        ${CMAKE_CURRENT_SOURCE_DIR}/${pkg} ${CMAKE_BINARY_DIR}/lib/${name})
    endforeach()

    foreach(script ${${pkg_name}_SCRIPTS})
      get_filename_component(name ${script} NAME)
      if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${script})
        message(FATAL_ERROR "
   Script ${name} as listed in setup.py of ${pkg_name} doesn't exist!!!
")
      endif()
      if(NOT EXISTS ${CMAKE_BINARY_DIR}/bin/${name})
        message(STATUS "   Making toplevel symlink for python script ${name}")
        execute_process(COMMAND /bin/ln -sf
          ${CMAKE_CURRENT_SOURCE_DIR}/${script} ${CMAKE_BINARY_DIR}/bin/${name})
      endif()
    endforeach()

  endif()

endfunction()

stamp(${catkin_EXTRAS_DIR}/interrogate_setup_dot_py.py)
