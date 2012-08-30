# toplevel CMakeLists.txt for a catkin workspace
# catkin/cmake/toplevel.cmake

cmake_minimum_required(VERSION 2.8)

# DISABLED optionally provide a cmake file in the workspace to override arbitrary stuff
#include(workspace.cmake OPTIONAL)

# list of unique workspaces based on CATKIN_WORKSPACES and CMAKE_PREFIX_PATH
# considers CMAKE_PREFIX_PATH for the case that setup.sh has not been sourced
# the same list is built in cmake/catkinConfig.cmake.in in the case toplevel.cmake is not used
set(CATKIN_WORKSPACES "")
foreach(workspace $ENV{CATKIN_WORKSPACES})
  string(REGEX REPLACE ":.*" "" workspace ${workspace})
  list(FIND CATKIN_WORKSPACES ${workspace} _index)
  if(_index EQUAL -1)
    list(APPEND CATKIN_WORKSPACES ${workspace})
  endif()
endforeach()
foreach(path ${CMAKE_PREFIX_PATH})
  if(EXISTS ${path}/CATKIN_WORKSPACE)
    list(FIND CATKIN_WORKSPACES ${path} _index)
    if(_index EQUAL -1)
      list(APPEND CATKIN_WORKSPACES ${path})
    endif()
  endif()
endforeach()

# include catkin directly or via find_package()
if(IS_DIRECTORY ${CMAKE_SOURCE_DIR}/catkin)
  add_subdirectory(catkin)
else()
  find_package(catkin REQUIRED
    NO_POLICY_SCOPE
    PATHS ${CATKIN_WORKSPACES}
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
endif()

catkin_workspace()
