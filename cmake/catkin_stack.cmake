#
# Process :ref:`stack.xml` from ``CMAKE_CURRENT_SOURCE_DIR`` and
# make several information available to CMake.
#
# .. note:: It must be called once in each stack's CMakeLists.txt,
#   before any calls to `catkin_project()`, to ensure that
#   auto-generated CMake and pkg-config files contain correct version
#   information.
#
# It installs ``stack.xml`` to ``share/${CATKIN_CURRENT_STACK}``.
#
# :outvar CATKIN_CURRENT_STACK: the stack name
# :outvar <stackname>_VERSION: the version number
# :outvar <stackname>_MAINTAINER: the name and email of the maintainer(s)
# :outvar <stackname>_DEPENDS: the build dependencies
#
# @public
#
function(catkin_stack)
  debug_message(10 "catkin_stack() called in file ${CMAKE_CURRENT_LIST_FILE}")

  # verify that no arguments are passed
  if(ARGN)
    message(FATAL_ERROR "catkin_stack() does not support arguments")
  endif()

  # ensure that function is not called multiple times per stack
  if(_CATKIN_CURRENT_STACK_INITIALIZED_)
    message(FATAL_ERROR "catkin_stack(): in '${CMAKE_CURRENT_LIST_FILE}', CATKIN_CURRENT_STACK is already set (to: ${CATKIN_CURRENT_STACK}).  This should not be the case.")
  endif()
  set(_CATKIN_CURRENT_STACK_INITIALIZED_)

  # stamp and parse stack.xml once
  if(NOT CATKIN_CURRENT_STACK)
    stamp(${CMAKE_CURRENT_SOURCE_DIR}/stack.xml)
    safe_execute_process(COMMAND ${PYTHON_EXECUTABLE}
      ${catkin_EXTRAS_DIR}/parse_stack_xml.py
      ${CMAKE_CURRENT_SOURCE_DIR}/stack.xml
      ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/stack.cmake)
  endif()
  # load extracted variable into cmake
  include(${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/stack.cmake)

  # install stack.xml
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/stack.xml
    DESTINATION share/${CATKIN_CURRENT_STACK})
endfunction()
