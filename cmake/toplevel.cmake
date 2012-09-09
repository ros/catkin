# toplevel CMakeLists.txt for a catkin workspace
# catkin/cmake/toplevel.cmake

cmake_minimum_required(VERSION 2.8)

# optionally provide a cmake file in the workspace to override arbitrary stuff
include(workspace.cmake OPTIONAL)

set(CATKIN_TOPLEVEL TRUE)

# include catkin directly or via find_package()
if(EXISTS "${CMAKE_SOURCE_DIR}/catkin/cmake/all.cmake" AND EXISTS "${CMAKE_SOURCE_DIR}/catkin/CMakeLists.txt")
  set(catkin_EXTRAS_DIR "${CMAKE_SOURCE_DIR}/catkin/cmake")
  # include all.cmake without add_subdirectory to let it operate in same scope
  include(catkin/cmake/all.cmake NO_POLICY_SCOPE)
  add_subdirectory(catkin)

else()
  # list of unique build- and installspaces
  set(catkin_search_path "")
  foreach(workspace $ENV{CATKIN_WORKSPACES})
    string(REGEX REPLACE ":.*" "" workspace ${workspace})
    list(APPEND catkin_search_path ${workspace})
  endforeach()
  list(REMOVE_DUPLICATES catkin_search_path)

  # use either CMAKE_PREFIX_PATH explicitly passed to CMake as a command line argument
  # or CMAKE_PREFIX_PATH from the environment
  if(NOT DEFINED CMAKE_PREFIX_PATH)
    set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
  endif()

  # search for catkin in all workspaces and CMAKE_PREFIX_PATH
  set(CATKIN_TOPLEVEL_FIND_PACKAGE TRUE)
  find_package(catkin REQUIRED
    NO_POLICY_SCOPE
    PATHS ${catkin_search_path} ${CMAKE_PREFIX_PATH}
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
  unset(CATKIN_TOPLEVEL_FIND_PACKAGE)
endif()

catkin_workspace()
