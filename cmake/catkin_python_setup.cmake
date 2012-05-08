# optionally give relative path to setup.py file
function(catkin_python_setup)
  check_unused_arguments("catkin_python_setup" "${ARGN}")

  assert(PROJECT_NAME)

  # mark that catkin_python_setup() was called in order to disable installation of gen/py stuff in generate_messages()
  set(${PROJECT_NAME}_CATKIN_PYTHON_SETUP TRUE PARENT_SCOPE)
  if(${PROJECT_NAME}_GENERATE_MESSAGES)
    message(FATAL_ERROR "generate_messages() must be called after catkin_python_setup() in project ${PROJECT_NAME}")
  endif()

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
    if(MSVC)
      # Need to convert install prefix to native path for python setuptools --prefix (its fussy about \'s)
      file(TO_NATIVE_PATH ${CMAKE_INSTALL_PREFIX} PYTHON_INSTALL_PREFIX)
      set(INSTALL_SCRIPT
        ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/python_distutils_install.bat)
      configure_file(${catkin_EXTRAS_DIR}/templates/python_distutils_install.bat.in
        ${INSTALL_SCRIPT}
        @ONLY)
    else()
      set(INSTALL_SCRIPT
        ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/python_distutils_install.sh)
      configure_file(${catkin_EXTRAS_DIR}/templates/python_distutils_install.sh.in
        ${INSTALL_SCRIPT}
        @ONLY)
    endif()
    
    configure_file(${catkin_EXTRAS_DIR}/templates/safe_execute_install.cmake.in
      ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/safe_execute_install.cmake)

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
    safe_execute_process(COMMAND
      ${CMD}
      )
    include(${${pkg_name}_BINARY_DIR}/${PATH_TO_SETUP_PY}setup_py_interrogation.cmake)

    if(${pkg_name}_PACKAGES)
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
    endif()

    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    foreach(script ${${pkg_name}_SCRIPTS})
      get_filename_component(name ${script} NAME)
      if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${script})
        message(FATAL_ERROR "
   Script ${name} as listed in ${SETUP_PY_FILE} of ${pkg_name} doesn't exist!!!
")
      endif()
      #message(STATUS "   Making toplevel forward script for python script ${name}")
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
