function(catkin_workspace)
  debug_message(10 "catkin_workspace() called in file '${CMAKE_CURRENT_LIST_FILE}'")

  # set global output directories for artifacts and create them if necessary
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${catkin_BUILD_PREFIX}/lib)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${catkin_BUILD_PREFIX}/lib)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${catkin_BUILD_PREFIX}/bin)
  if(NOT IS_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
    file(MAKE_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
  endif()
  if(NOT IS_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
    file(MAKE_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
  endif()

  # tools/libraries.cmake
  configure_shared_library_build_settings()

  # add include folder from all workspaces
  foreach(workspace $ENV{CATKIN_WORKSPACES})
    string(REGEX REPLACE ":.*" "" workspace ${workspace})
    include_directories(${workspace}/include)
  endforeach()
  include_directories(BEFORE ${catkin_BUILD_PREFIX}/include)

  set(CATKIN_WHITELIST_STACKS "" CACHE STRING "List of ';' separated stacks to build")
  set(CATKIN_BLACKLIST_STACKS "" CACHE STRING "List of ';' separated stacks to exclude")

  assert(catkin_EXTRAS_DIR)
  em_expand(
    ${catkin_EXTRAS_DIR}/templates/sort_stacks_topologically.context.py.in
    ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/sort_stacks_topologically.py
    ${catkin_EXTRAS_DIR}/em/sort_stacks_topologically.cmake.em
    ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/sort_stacks_topologically.cmake
    )
  debug_message(10 "catkin_workspace() including sort_stacks_topologically.cmake")
  include(${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/sort_stacks_topologically.cmake)

  if(CATKIN_TOPOLOGICALLY_SORTED_STACKS)
    message(STATUS "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    message(STATUS "~~  traversing stacks in dependency order  ~~")
    message(STATUS "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    message(STATUS "build order of stacks:")
    foreach(stack_path ${CATKIN_TOPOLOGICALLY_SORTED_STACKS})
      message(STATUS "- ${stack_path}")
    endforeach()

    foreach(stack_path ${CATKIN_TOPOLOGICALLY_SORTED_STACKS})
      get_filename_component(folder ${stack_path} NAME)
      message(STATUS "+++ add_subdirectory(${folder})")
      set(CATKIN_CURRENT_STACK "" CACHE INTERNAL "" FORCE)
      stamp(${stack_path}/stack.xml)
      add_subdirectory(${stack_path})
    endforeach()
  endif()
endfunction()
