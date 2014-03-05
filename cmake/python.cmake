# the CMake variable PYTHON_INSTALL_DIR has the same value as the Python function catkin.builder.get_python_install_dir()

set(PYTHON_VERSION "" CACHE STRING "Specify specific Python version to use ('major.minor' or 'major')")
if(PYTHON_VERSION)
  set(PythonInterp_FIND_VERSION "${PYTHON_VERSION}")
endif()

find_package(PythonInterp REQUIRED)
message(STATUS "Using PYTHON_EXECUTABLE: ${PYTHON_EXECUTABLE}")

# in CMake 2.8.6 and older PythonInterp does not set the version variables
if(NOT PYTHON_VERSION_MAJOR OR NOT PYTHON_VERSION_MINOR)
  message(STATUS "Alternative approach to get Python version since find_package(PythonInterp) does not return it")
  execute_process(COMMAND "${PYTHON_EXECUTABLE}" "-c" "from sys import version_info; print('%u;%u' % (version_info[0], version_info[1]))"
    RESULT_VARIABLE _res
    OUTPUT_VARIABLE _out
    ERROR_VARIABLE _err
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_STRIP_TRAILING_WHITESPACE
  )
  if(NOT _res EQUAL 0)
    message(FATAL_ERROR "Determine Python version failed")
  endif()
  list(GET _out 0 PYTHON_VERSION_MAJOR)
  list(GET _out 1 PYTHON_VERSION_MINOR)
endif()
message(STATUS "Python version: ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")

set(_PYTHON_PATH_VERSION_SUFFIX "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")

set(enable_setuptools_deb_layout OFF)
if(EXISTS "/etc/debian_version")
  set(enable_setuptools_deb_layout ON)
endif()
option(SETUPTOOLS_DEB_LAYOUT "Enable debian style python package layout" ${enable_setuptools_deb_layout})

if(SETUPTOOLS_DEB_LAYOUT)
  message(STATUS "Using Debian Python package layout")
  set(PYTHON_PACKAGES_DIR dist-packages)
  set(SETUPTOOLS_ARG_EXTRA "--install-layout=deb")
  # use major version only when installing 3.x with debian layout
  if("${PYTHON_VERSION_MAJOR}" STREQUAL "3")
    set(_PYTHON_PATH_VERSION_SUFFIX "${PYTHON_VERSION_MAJOR}")
  endif()
else()
  message(STATUS "Using default Python package layout")
  set(PYTHON_PACKAGES_DIR site-packages)
  # setuptools is fussy about windows paths, make sure the install prefix is in native format
  file(TO_NATIVE_PATH "${CMAKE_INSTALL_PREFIX}" SETUPTOOLS_INSTALL_PREFIX)
endif()

if(NOT WIN32)
  set(PYTHON_INSTALL_DIR lib/python${_PYTHON_PATH_VERSION_SUFFIX}/${PYTHON_PACKAGES_DIR}
    CACHE INTERNAL "This needs to be in PYTHONPATH when 'setup.py install' is called.  And it needs to match.  But setuptools won't tell us where it will install things.")
else()
  # Windows setuptools installs to lib/site-packages not lib/python2.7/site-packages
  set(PYTHON_INSTALL_DIR lib/${PYTHON_PACKAGES_DIR}
    CACHE INTERNAL "This needs to be in PYTHONPATH when 'setup.py install' is called.  And it needs to match.  But setuptools won't tell us where it will install things.")
endif()
