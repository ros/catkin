find_package(PythonInterp)

macro(wg_python)
  if(EXISTS ${PROJECT_SOURCE_DIR}/setup.sh)
    execute_process(${PYTHON_EXECUTABLE} ${PROJECT_SOURCE_DIR}/setup.sh -q develop 
      --install-dir ${CMAKE_BINARY_DIR}
      )
  endif()

  install(CODE 
    "execute_process(
        COMMAND ${PYTHON_EXECUTABLE} ${PROJECT_SOURCE_DIR}/setup.py install --old-and-unmanageable --prefix ${CMAKE_INSTALL_PREFIX}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})")

endmacro()