# the CMake variable PYTHON_INSTALL_DIR has the same value as the Python function catkin.builder.get_python_install_dir()

set(PYTHON_VERSION "" CACHE STRING "Specify specific Python version to use ('major.minor' or 'major')")
if(PYTHON_VERSION)
  set(PythonInterp_FIND_VERSION "${PYTHON_VERSION}")
endif()

find_package(PythonInterp REQUIRED)
message(STATUS "Using PYTHON_EXECUTABLE: ${PYTHON_EXECUTABLE}")

set(_PYTHON_PATH_VERSION_SUFFIX "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")

# setuptools is fussy about windows paths, make sure the install prefix is in native format
file(TO_NATIVE_PATH "${CMAKE_INSTALL_PREFIX}" SETUPTOOLS_INSTALL_PREFIX)

# PYTHON_INSTALL_DIR needs to be in PYTHONPATH when 'setup.py install'
# is called, and it needs to match. setuptools won't tell us where it will
# install things, so we'll ask Python. Becuase this has to match
# python/catkin/builder.py exactly, we let Python handle the work.
execute_process(COMMAND "${PYTHON_EXECUTABLE}"
  "-c" "import site; import os; python_install_dir=os.sep.join(site.getsitepackages()[1 if os.name == 'nt' else 0].split(os.sep)[-2 if os.name == 'nt' else -3:]); print(python_install_dir)"
  RESULT_VARIABLE _res
  OUTPUT_VARIABLE PYTHON_INSTALL_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT _res EQUAL 0)
  message(FATAL_ERROR "Determine PYTHON_INSTALL_DIR failed")
endif()
message(STATUS "Using PYTHON_INSTALL_DIR: " ${PYTHON_INSTALL_DIR})

