#
# Search all direct subfolders in the workspace for ``stack.xml``
# files.  Based on the dependencies specified in the
# ``build_depends`` tags it performs a topological sort and calls
# ``add_subdirectory()`` for each directory.
#
# The functions is only called in catkin's ``toplevel.cmake``, which
# is usually symlinked to the workspace root directory (which
# contains multiple stacks).
#
function(catkin_workspace)
  debug_message(10 "catkin_workspace() called in file '${CMAKE_CURRENT_LIST_FILE}'")

  # set global output directories for artifacts and create them if necessary
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CATKIN_BUILD_PREFIX}/lib)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CATKIN_BUILD_PREFIX}/lib)
  if(NOT IS_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
    file(MAKE_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
  endif()

  # tools/libraries.cmake
  configure_shared_library_build_settings()

  # get include folders from all build- and installspaces
  foreach(workspace ${CATKIN_WORKSPACES})
    string(REGEX REPLACE ":.*" "" workspace ${workspace})
    include_directories(${workspace}/include)
  endforeach()
  include_directories(BEFORE ${CATKIN_BUILD_PREFIX}/include)

  set(CATKIN_WHITELIST_STACKS "" CACHE STRING "List of ';' separated stacks to build")
  set(CATKIN_BLACKLIST_STACKS "" CACHE STRING "List of ';' separated stacks to exclude")

  assert(catkin_EXTRAS_DIR)
  em_expand(
    ${catkin_EXTRAS_DIR}/templates/order_projects.context.py.in
    ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/order_projects.py
    ${catkin_EXTRAS_DIR}/em/order_projects.cmake.em
    ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/order_projects.cmake
    )
  debug_message(10 "catkin_workspace() including order_projects.cmake")
  include(${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/order_projects.cmake)

  if(CATKIN_ORDERED_PROJECTS)
    message(STATUS "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    message(STATUS "~~  traversing projects in topological order:")
    foreach(name ${CATKIN_ORDERED_PROJECTS})
      message(STATUS "~~  - ${name}")
    endforeach()
    message(STATUS "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

    list(LENGTH CATKIN_ORDERED_PROJECTS count)
    math(EXPR range "${count} - 1")
    foreach(index RANGE ${range})
      list(GET CATKIN_ORDERED_PROJECTS ${index} name)
      list(GET CATKIN_ORDERED_PROJECT_PATHS ${index} path)
      message(STATUS "+++ add_subdirectory(${path})")
      unset(CATKIN_CURRENT_STACK)
      stamp(${path}/stack.xml)
      add_subdirectory(${path})
    endforeach()
  endif()
endfunction()
