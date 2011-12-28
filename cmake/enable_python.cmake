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
endfunction()

