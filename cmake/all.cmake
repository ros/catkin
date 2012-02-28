include(${catkin_EXTRAS_DIR}/log.cmake)
include(${catkin_EXTRAS_DIR}/assert.cmake)
assert(catkin_EXTRAS_DIR)

if (CATKIN_ALL_INCLUDED)
  return()
endif()
set(CATKIN_ALL_INCLUDED)

if(catkin_BINARY_DIR)
  set(CATKIN_CONTEXT_FILE ${catkin_BINARY_DIR}/catkin-context.py
    CACHE INTERNAL "catkin context file")
  if(WIN32 AND NOT CYGWIN)
    set(CATKIN_ENV ${CMAKE_BINARY_DIR}/env.bat CACHE INTERNAL "catkin env")
    message(STATUS "Shell environment is defined in build directory ${CMAKE_BINARY_DIR}/env.bat")
  else()
    set(CATKIN_ENV ${CMAKE_BINARY_DIR}/env.sh CACHE INTERNAL "catkin env")
    message(STATUS "Shell environment is defined in build directory ${CMAKE_BINARY_DIR}/env.sh")
  endif()
else()
  set(CATKIN_CONTEXT_FILE ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/catkin-context.py
    CACHE INTERNAL "catkin context file")
  set(CATKIN_ENV ${catkin_INSTALL_PREFIX}/env.sh
    CACHE INTERNAL "catkin env")
  message(STATUS "Shell environment is defined in catkin installation at ${catkin_INSTALL_PREFIX}/env.sh")
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
    langs
    libraries
    rosbuild_compat
    platform/lsb
    platform/ubuntu
    tools/doxygen
    tools/rt
    tools/threads
    tools/gtest
    tests
    )
  include(${catkin_EXTRAS_DIR}/${f}.cmake)
endforeach()

#
# These get generated no matter what.
#
catkin_generic_hooks()
