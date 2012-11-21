include(${catkin_EXTRAS_DIR}/log.cmake)
include(${catkin_EXTRAS_DIR}/assert.cmake)
assert(catkin_EXTRAS_DIR)

if (CATKIN_ALL_INCLUDED)
  return()
endif()
set(CATKIN_ALL_INCLUDED)

if( CMAKE_HOST_UNIX ) # true for linux, apple, mingw-cross and cygwin
  set(CATKIN_ENV ${CMAKE_BINARY_DIR}/env.sh CACHE INTERNAL "catkin env")
else()
  set(CATKIN_ENV ${CMAKE_BINARY_DIR}/env.bat CACHE INTERNAL "catkin env")
endif()
if(catkin_BINARY_DIR)
  set(CATKIN_CONTEXT_FILE ${catkin_BINARY_DIR}/catkin-context.py
    CACHE INTERNAL "catkin context file")
else()
  set(CATKIN_CONTEXT_FILE ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/catkin-context.py
    CACHE INTERNAL "catkin context file")
endif()
configure_file(${catkin_EXTRAS_DIR}/templates/catkin-context.in
  ${CATKIN_CONTEXT_FILE}
  )

foreach(f
    check_unused_arguments
    python
    empy
    stamp
    set_once
    safe_execute_process
    parse_arguments
    em_expand
    find_program_required
    install_matching_to_share
    catkin_stack
    catkin_project
    catkin_workspace
    catkin_python_setup
    catkin_add_env_hooks
    rosbuild_compat
    platform/lsb
    platform/ubuntu
    platform/windows
    tools/doxygen
    tools/libraries
    tools/rt
    tools/threads
    tools/gtest
    future
    )
  include(${catkin_EXTRAS_DIR}/${f}.cmake)
endforeach()
if(catkin_SOURCE_DIR)
  include(${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/tests.cmake)
else()
  include(${catkin_EXTRAS_DIR}/tests.cmake)
endif()
#
# These get generated no matter what.
#
catkin_generic_hooks()
