# toplevel CMakeLists.txt for a catkin workspace
# catkin/cmake/toplevel.cmake

cmake_minimum_required(VERSION 2.8)

# DISABLED optionally provide a cmake file in the workspace to override arbitrary stuff
#include(workspace.cmake OPTIONAL)

# append all workspaces to the CMAKE_PREFIX_PATH
foreach(workspace $ENV{CATKIN_WORKSPACES})
  string(REGEX REPLACE ":.*" "" workspace ${workspace})
  list(FIND CMAKE_PREFIX_PATH ${workspace} _index)
  if(_index EQUAL -1)
    list(APPEND CMAKE_PREFIX_PATH ${workspace})
  endif()
endforeach()

# include catkin directly or via find_package()
if(IS_DIRECTORY ${CMAKE_SOURCE_DIR}/catkin)
  add_subdirectory(catkin)
else()
  find_package(catkin REQUIRED PATHS ${CMAKE_PREFIX_PATH} NO_POLICY_SCOPE)
endif()

catkin_workspace()
