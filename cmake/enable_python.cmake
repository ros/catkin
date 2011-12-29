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

    install(CODE "message(COMMAND = ${CMD})")
    install(CODE "execute_process(COMMAND ${INSTALL_SCRIPT})")
  endif()

  if (EXISTS ${${pkg_name}_SOURCE_DIR}/setup.py)
    stamp(${${pkg_name}_SOURCE_DIR}/setup.py)

    execute_process(COMMAND
      ${CATKIN_ENV} ${PYTHON_EXECUTABLE}
      ${catkin_EXTRAS_DIR}/interrogate_setup_dot_py.py
      ${pkg_name}
      ${${pkg_name}_SOURCE_DIR}/setup.py
      ${${pkg_name}_BINARY_DIR}/setup_py_interrogation.cmake)

    include(${${pkg_name}_BINARY_DIR}/setup_py_interrogation.cmake)
  endif()
  foreach(script ${${pkg_name}_SCRIPTS})
    get_filename_component(name ${script} NAME)
    if(NOT EXISTS ${CMAKE_BINARY_DIR}/bin/${name})
      execute_process(COMMAND /bin/ln -s ${CMAKE_CURRENT_SOURCE_DIR}/${script} ${CMAKE_BINARY_DIR}/bin/${name})
    endif()
  endforeach()



endfunction()

stamp(${catkin_EXTRAS_DIR}/interrogate_setup_dot_py.py)
