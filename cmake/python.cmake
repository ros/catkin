find_package(PythonInterp)
execute_process(COMMAND ${PYTHON_EXECUTABLE} ${catkin_EXTRAS_DIR}/python_version.py
  OUTPUT_VARIABLE PYTHON_VERSION_XDOTY
  OUTPUT_STRIP_TRAILING_WHITESPACE)

set(PYTHON_VERSION_XDOTY ${PYTHON_VERSION_XDOTY} CACHE STRING "python version")

# this should actually be for anything non-dist-packages
if(APPLE)
  set(PYTHON_PACKAGES_DIR site-packages CACHE STRING "dist-packages or site-packages")
  set(SETUPTOOLS_ARG_EXTRA "" CACHE STRING "extra arguments to setuptools")
else()
  set(PYTHON_PACKAGES_DIR dist-packages CACHE STRING "dist-packages or site-packages")
  set(SETUPTOOLS_ARG_EXTRA "--install-layout=deb" CACHE STRING "extra arguments to setuptools")
endif()

set(INSTALLED_PYTHONPATH ${CMAKE_INSTALL_PREFIX}/lib/python${PYTHON_VERSION_XDOTY}/${PYTHON_PACKAGES_DIR}
  CACHE INTERNAL "This needs to be in pythonpath when setup.py install is called.  And it needs to match.  But setuptools won't tell us where it will install things.")


