function(enable_python pkg_name)
  set(PACKAGE_NAME ${pkg_name})
  configure_file(${catkin_EXTRAS_DIR}/__init__.py.in
    ${CMAKE_BINARY_DIR}/gen/py/${pkg_name}/__init__.py
    @ONLY)
  if(EXISTS ${${pkg_name}_SOURCE_DIR}/setup.py)
    assert(INSTALLED_PYTHONPATH)
    set(CMD "/usr/bin/env PYTHONPATH=${INSTALLED_PYTHONPATH} ${PYTHON_EXECUTABLE} setup.py install --root=$DESTDIR --install-layout=deb --prefix=${CMAKE_INSTALL_PREFIX} WORKING_DIRECTORY ${${pkg_name}_SOURCE_DIR}")
    install(CODE "message(COMMAND = ${CMD})")
    install(CODE "execute_process(COMMAND ${CMD})")
  endif()
endfunction()