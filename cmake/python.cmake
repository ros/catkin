# the CMake variable PYTHON_INSTALL_DIR has the same value as the Python function catkin.builder.get_python_install_dir()

find_package(PythonInterp REQUIRED)
execute_process(COMMAND ${PYTHON_EXECUTABLE} ${catkin_EXTRAS_DIR}/python_version.py
  OUTPUT_VARIABLE PYTHON_VERSION_XDOTY
  OUTPUT_STRIP_TRAILING_WHITESPACE)

set(PYTHON_VERSION_XDOTY ${PYTHON_VERSION_XDOTY} CACHE STRING "Python version")

set(python_use_debian_layout FALSE)
if(EXISTS "/etc/debian_version")
  set(python_use_debian_layout TRUE)
endif()

if(python_use_debian_layout)
  set(PYTHON_PACKAGES_DIR dist-packages)
  set(SETUPTOOLS_ARG_EXTRA "--install-layout=deb")
else()
  set(PYTHON_PACKAGES_DIR site-packages)
  # setuptools is fussy about windows paths, make sure the install prefix is in native format
  file(TO_NATIVE_PATH "${CMAKE_INSTALL_PREFIX}" SETUPTOOLS_INSTALL_PREFIX)
endif()

if(NOT WIN32)
  set(PYTHON_INSTALL_DIR lib/python${PYTHON_VERSION_XDOTY}/${PYTHON_PACKAGES_DIR}
    CACHE INTERNAL "This needs to be in PYTHONPATH when 'setup.py install' is called.  And it needs to match.  But setuptools won't tell us where it will install things.")
else()
  # Windows setuptools installs to lib/site-packages not lib/python2.7/site-packages 
  set(PYTHON_INSTALL_DIR lib/${PYTHON_PACKAGES_DIR}
    CACHE INTERNAL "This needs to be in PYTHONPATH when 'setup.py install' is called.  And it needs to match.  But setuptools won't tell us where it will install things.")
endif()
