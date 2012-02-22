# optionally give relative path to setup.py file
function(catkin_python_setup)

  assert(PROJECT_NAME)

  # Use PROJECT_NAME as pkg_name
  set(pkg_name ${PROJECT_NAME})

  if(${ARGC} GREATER 1)
    message(WARNING "catkin_python_setup() takes only one optional argument, update project ${pkg_name}")
  endif()
  set(PATH_TO_SETUP_PY "")
  if(${ARGC} EQUAL 1)
    set(PATH_TO_SETUP_PY "${ARGN}/")
  endif()
  set(SETUP_PY_FILE "${PATH_TO_SETUP_PY}setup.py")
  if(NOT("${PATH_TO_SETUP_PY}" STREQUAL ""))
    string(REPLACE "." "_" PATH_TO_SETUP_PY ${PATH_TO_SETUP_PY})
  endif()

  if(NOT EXISTS ${${pkg_name}_SOURCE_DIR}/${SETUP_PY_FILE})
    message(WARNING "catkin_python_setup() called without ${SETUP_PY_FILE} in project ${pkg_name}")
  endif()

  if(EXISTS ${${pkg_name}_SOURCE_DIR}/${SETUP_PY_FILE})
    assert(INSTALLED_PYTHONPATH)
    set(INSTALL_CMD_WORKING_DIRECTORY ${${pkg_name}_SOURCE_DIR})
    set(INSTALL_SCRIPT
      ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/python_distutils_install.sh)

    configure_file(${catkin_EXTRAS_DIR}/templates/python_distutils_install.sh.in
      ${INSTALL_SCRIPT}
      @ONLY)

    configure_file(${catkin_EXTRAS_DIR}/templates/safe_execute_install.cmake.in
      ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/safe_execute_install.cmake)

    install(CODE "message(COMMAND = ${CMD})")
    install(SCRIPT ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/safe_execute_install.cmake)

    stamp(${${pkg_name}_SOURCE_DIR}/${SETUP_PY_FILE})

    set(CMD
      ${CATKIN_ENV} ${PYTHON_EXECUTABLE}
      ${catkin_EXTRAS_DIR}/interrogate_setup_dot_py.py
      ${pkg_name}
      ${${pkg_name}_SOURCE_DIR}/${SETUP_PY_FILE}
      ${${pkg_name}_BINARY_DIR}/${PATH_TO_SETUP_PY}setup_py_interrogation.cmake
      )

    # message("IN ${pkg_name}:  ${CMD}")
    execute_process(COMMAND
      ${CMD}
      RESULT_VARIABLE RES
      )
    if (RES)
      message(FATAL_ERROR "Attempt to interrogate ${SETUP_PY_FILE} of project ${pkg_name} returned ${RES}")
    endif()
    include(${${pkg_name}_BINARY_DIR}/${PATH_TO_SETUP_PY}setup_py_interrogation.cmake)

    list(LENGTH ${pkg_name}_PACKAGES pkgs_count)
    math(EXPR pkgs_range "${pkgs_count} - 1")
    foreach(index RANGE ${pkgs_range})
      list(GET ${pkg_name}_PACKAGES ${index} pkg)
      list(GET ${pkg_name}_PACKAGE_DIRS ${index} pkg_dir)
      get_filename_component(name ${pkg_dir} NAME)
      if(NOT ("${pkg}" STREQUAL "${name}"))
        message(FATAL_ERROR "The package name ${pkg} differs from the basename of the path ${pkg_dir} in project ${PROJECT_NAME}")
      endif()
      get_filename_component(path ${pkg_dir} PATH)
      set(PACKAGE_PYTHONPATH ${CMAKE_CURRENT_SOURCE_DIR}/${path})
      configure_file(${catkin_EXTRAS_DIR}/templates/__init__.py.in
        ${CMAKE_BINARY_DIR}/gen/py/${pkg}/__init__.py
        @ONLY)
    endforeach()

    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    foreach(script ${${pkg_name}_SCRIPTS})
      get_filename_component(name ${script} NAME)
      if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${script})
        message(FATAL_ERROR "
   Script ${name} as listed in ${SETUP_PY_FILE} of ${pkg_name} doesn't exist!!!
")
      endif()
      message(STATUS "   Making toplevel forward script for python script ${name}")
      set(PYTHON_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/${script})
      configure_file(${catkin_EXTRAS_DIR}/templates/script.py.in
        ${CMAKE_BINARY_DIR}/bin/${name}
        @ONLY)
    endforeach()

  endif()

endfunction()

stamp(${catkin_EXTRAS_DIR}/interrogate_setup_dot_py.py)

function(catkin_export_python)
  message(WARNING "CMAKE macro catkin_export_python replaced by catkin_python_setup")
  catkin_python_setup()
endfunction()

function(enable_python)
  message(WARNING "CMAKE macro enable_python replaced by catkin_export_python")
  catkin_export_python(${ARGN})
endfunction()
