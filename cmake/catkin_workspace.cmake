#
# Search all subfolders in the workspace for ``package.xml`` files.
# Based on the dependencies specified in the ``build_depends`` and
# ``buildtool_depends`` tags it performs a topological sort and calls
# ``add_subdirectory()`` for each directory.
#
# The functions is only called in catkin's ``toplevel.cmake``, which
# is usually symlinked to the workspace root directory (which
# contains multiple packages).
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

  # get include folders from all workspaces
  foreach(workspace ${CATKIN_WORKSPACES})
    include_directories(${workspace}/include)
  endforeach()
  include_directories(BEFORE ${CATKIN_BUILD_PREFIX}/include)

  set(CATKIN_WHITELIST_PACKAGES "" CACHE STRING "List of ';' separated packages to build")
  set(CATKIN_BLACKLIST_PACKAGES "" CACHE STRING "List of ';' separated packages to exclude")

  assert(catkin_EXTRAS_DIR)
  em_expand(
    ${catkin_EXTRAS_DIR}/templates/order_packages.context.py.in
    ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/order_packages.py
    ${catkin_EXTRAS_DIR}/em/order_packages.cmake.em
    ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/order_packages.cmake
    )
  debug_message(10 "catkin_workspace() including order_packages.cmake")
  include(${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/order_packages.cmake)

  if(CATKIN_ORDERED_PACKAGES)
    message(STATUS "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    message(STATUS "~~  traversing packages in topological order:")
    foreach(name ${CATKIN_ORDERED_PACKAGES})
      message(STATUS "~~  - ${name}")
    endforeach()
    message(STATUS "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

    list(LENGTH CATKIN_ORDERED_PACKAGES count)
    math(EXPR range "${count} - 1")
    foreach(index RANGE ${range})
      # ensure that no current package name is set
      unset(_CATKIN_CURRENT_PACKAGE)

      list(GET CATKIN_ORDERED_PACKAGE_PATHS ${index} path)
      list(GET CATKIN_ORDERED_PACKAGES_IS_META ${index} is_meta)
      list(GET CATKIN_ORDERED_PACKAGES ${index} name)
      if(NOT ${is_meta})
        message(STATUS "+++ add_subdirectory(${path})")
        add_subdirectory(${path})
      else()
        message(STATUS "+++ skipping metapackage '${path}'")
      endif()
    endforeach()
  endif()
endfunction()
