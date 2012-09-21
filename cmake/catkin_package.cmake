#
# Process :ref:`package.xml` from ``CMAKE_CURRENT_SOURCE_DIR`` and
# make several information available to CMake.
#
# .. note:: It must be called once in each package, after calling
#   project() where the project name must match the package name and
#   before calling `catkin_package_export()`, to ensure that
#   auto-generated CMake and pkg-config files contain correct version
#   information.  Best practice is to call this macro early in your
#   root CMakeLists.txt, immediately after calling ``project()`` and
#   ``find_package(catkin REQUIRED)``.
#
# It installs ``package.xml`` to ``share/${PROJECT_NAME}``.
#
# :outvar <packagename>_VERSION: the version number
# :outvar <packagename>_MAINTAINER: the name and email of the maintainer(s)
# :outvar <packagename>_DEPENDS: the build dependencies
#
# @public
#
macro(catkin_package)
  debug_message(10 "catkin_package() called in file ${CMAKE_CURRENT_LIST_FILE}")

  # verify that no arguments are passed
  if(ARGN)
    message(FATAL_ERROR "catkin_package() does not support arguments")
  endif()

  # verify that project() has been called before
  if(NOT PROJECT_NAME)
    message(FATAL_ERROR "catkin_package() PROJECT_NAME is not set. You must call project() before you can call catkin_package().")
  endif()

  # ensure that function is not called multiple times per package
  if(DEFINED _CATKIN_CURRENT_PACKAGE)
    message(FATAL_ERROR "catkin_package(): in '${CMAKE_CURRENT_LIST_FILE}', _CATKIN_CURRENT_PACKAGE is already set (to: ${_CATKIN_CURRENT_PACKAGE}).  This should not be the case.")
  endif()

  # stamp and parse package.xml
  stamp(${CMAKE_CURRENT_SOURCE_DIR}/package.xml)
  safe_execute_process(COMMAND ${PYTHON_EXECUTABLE}
    ${catkin_EXTRAS_DIR}/parse_package_xml.py
    ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
    ${PROJECT_BINARY_DIR}/catkin_generated/package.cmake)
  # load extracted variable into cmake
  include(${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/package.cmake)

  # verify that the package name from package.xml equals the project() name
  if(NOT _CATKIN_CURRENT_PACKAGE STREQUAL PROJECT_NAME)
    message(FATAL_ERROR "catkin_package_export() package name in package.xml '${_CATKIN_CURRENT_PACKAGE}' does not match current PROJECT_NAME '${PROJECT_NAME}'.  You must call project() with the same name before.")
  endif()

  # install package.xml
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
    DESTINATION share/${PROJECT_NAME})

  # automatically chain dependencies for meta-packages
  if(${_CATKIN_CURRENT_PACKAGE_IS_META})
    set(_CATKIN_PACKAGE_EXPORT_CALLED_INTERNALLY TRUE)
    find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_DEPENDS})
    catkin_package_export(DEPENDS ${${PROJECT_NAME}_DEPENDS})
    unset(_CATKIN_PACKAGE_EXPORT_CALLED_INTERNALLY)
  endif()
endmacro()
