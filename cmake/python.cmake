find_package(PythonInterp)
execute_process(COMMAND ${PYTHON_EXECUTABLE} ${catkin_EXTRAS_DIR}/python_version.py
  OUTPUT_VARIABLE PYTHON_VERSION_XDOTY
  OUTPUT_STRIP_TRAILING_WHITESPACE)

set(PYTHON_VERSION_XDOTY ${PYTHON_VERSION_XDOTY} CACHE STRING "python version")
set(INSTALLED_PYTHONPATH ${CMAKE_INSTALL_PREFIX}/lib/python${PYTHON_VERSION_XDOTY}/dist-packages
  CACHE INTERNAL "This needs to be in pythonpath when setup.py install is called.  And it needs to match.  But setuptools won't tell us where it will install things.")


