#
# It installs the package.xml file of a metapackage.
#
# .. note:: It must be called once for each metapackage.  Best
#   practice is to call this macro early in your root CMakeLists.txt,
#   immediately after calling ``project()`` and
#   ``find_package(catkin REQUIRED)``.
#
# @public
#
function(catkin_metapackage)
  if(ARGN)
    message(FATAL_ERROR "catkin_metapackage() called with unused arguments: ${ARGN}")
  endif()

  debug_message(10 "catkin_metapackage() called in file ${CMAKE_CURRENT_LIST_FILE}")

  # verify that project() has been called before
  if(NOT PROJECT_NAME)
    message(FATAL_ERROR "catkin_metapackage() PROJECT_NAME is not set. You must call project() before calling catkin_metapackage().")
  endif()
  if(PROJECT_NAME STREQUAL "Project")
    message(FATAL_ERROR "catkin_metapackage() PROJECT_NAME is set to 'Project', which is not a valid project name. You must call project() before calling catkin_metapackage().")
  endif()

  cmake_parse_arguments(PROJECT "" "" "INCLUDE_DIRS;LIBRARIES;CATKIN_DEPENDS;DEPENDS;CFG_EXTRAS" ${ARGN})
  if(PROJECT_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "catkin_package() called with unused arguments: ${PROJECT_UNPARSED_ARGUMENTS}")
  endif()

  # call catkin_package_xml()
  if(${CMAKE_CURRENT_LIST_FILE} STREQUAL ${CMAKE_BINARY_DIR}/catkin_generated/metapackages/${PROJECT_NAME}/CMakeLists.txt)
    set(package_dir ${CMAKE_SOURCE_DIR}/${path})
    catkin_package_xml(DIRECTORY ${CMAKE_SOURCE_DIR}/${path})
  else()
    set(package_dir ${CMAKE_CURRENT_SOURCE_DIR})
    catkin_package_xml()
  endif()

  # install package.xml
  install(FILES ${package_dir}/package.xml
    DESTINATION share/${PROJECT_NAME}
  )
endfunction()
