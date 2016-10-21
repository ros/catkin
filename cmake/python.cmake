# the CMake variable PYTHON_INSTALL_DIR has the same value as the Python function catkin.builder.get_python_install_dir()

set(PYTHON_VERSION "" CACHE STRING "Specify specific Python version to use ('major.minor' or 'major')")
if(PYTHON_VERSION)
  set(PythonInterp_FIND_VERSION "${PYTHON_VERSION}")
endif()

find_package(PythonInterp REQUIRED)
message(STATUS "Using PYTHON_EXECUTABLE: ${PYTHON_EXECUTABLE}")

set(_PYTHON_PATH_VERSION_SUFFIX "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")

# setuptools is fussy about windows paths, make sure the install prefix is in native format
if(WIN32)
  file(TO_NATIVE_PATH "${CMAKE_INSTALL_PREFIX}" SETUPTOOLS_INSTALL_PREFIX)
endif()

# a precomputed PYTHON_INSTALL_DIR can be passed along with the matching PYTHON_EXECUTABLE here
# in order to avoid needing to call PYTHON_EXECUTABLE in the next block in each package
# the precomputed value is only taken if the PYTHON_EXECUTABLE matches what was found above
if(DEFINED PYTHON_INSTALL_DIR_TUPLE)
  string(REPLACE ":" ";" _tuple ${PYTHON_INSTALL_DIR_TUPLE})
  list(LENGTH _tuple _tuple_len)
  if(_tuple_len EQUAL 2)
    list(GET _tuple 0 _tuple_PYTHON_EXECUTABLE)
    list(GET _tuple 1 _tuple_PYTHON_INSTALL_DIR)
    if("${_tuple_PYTHON_EXECUTABLE} " STREQUAL "${PYTHON_EXECUTABLE} ")
      set(PYTHON_INSTALL_DIR "${_tuple_PYTHON_INSTALL_DIR}")
    else()
      message(WARNING
        "PYTHON_INSTALL_DIR_TUPLE is set, "
        "but the given PYTHON_EXECUTABLE '${_tuple_PYTHON_EXECUTABLE}' does not "
        "match the PYTHON_EXECUTABLE found by CMake '${PYTHON_EXECUTABLE}'.\n"
        "The precomputed value in the tuple will be ignored and Python will be "
        "invoked to get the correct PYTHON_INSTALL_DIR.")
    endif()
  else()
    message(WARNING
      "PYTHON_INSTALL_DIR_TUPLE is set, "
      "but the contents '${PYTHON_INSTALL_DIR_TUPLE}' are invalid.\n"
      "It should take the form of '<PYTHON_EXECUTABLE>:<PYTHON_INSTALL_DIR>' "
      "with the executable and install dir separated with a colon.")
  endif()
endif()

# this block determines the same value as the Python function get_python_install_dir() from python/catkin/builder.py
if(NOT DEFINED PYTHON_INSTALL_DIR)
  execute_process(COMMAND "${PYTHON_EXECUTABLE}"
    "-c" "from distutils.sysconfig import get_python_lib; import os; print(os.sep.join(get_python_lib().split(os.sep)[-2 if os.name == 'nt' else -3:]))"
    RESULT_VARIABLE _res
    OUTPUT_VARIABLE _var
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(NOT _res EQUAL 0)
    message(FATAL_ERROR "Failed to determine the PYTHON_INSTALL_DIR")
  endif()
  set(PYTHON_INSTALL_DIR "${_var}"
  CACHE INTERNAL "This needs to be in PYTHONPATH when 'setup.py install' is called otherwise setuptools will fail.")
endif()
message(STATUS "Using PYTHON_INSTALL_DIR: " ${PYTHON_INSTALL_DIR})

if(PYTHON_INSTALL_DIR MATCHES "[/\\]dist-packages$")
  set(SETUPTOOLS_ARG_EXTRA "--install-layout=deb")
endif()
