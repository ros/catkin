include(${catkin_EXTRAS_DIR}/dist.cmake)
include(${catkin_EXTRAS_DIR}/log.cmake)
include(${catkin_EXTRAS_DIR}/assert.cmake)
assert(catkin_EXTRAS_DIR)

if (CATKIN_ALL_INCLUDED)
  return()
else()
  set(CATKIN_ALL_INCLUDED)
endif()

foreach(f
    python
    empy
    stamp
    set_once
    safe_execute_process
    parse_arguments
    wg_python
    debian-util
    em_expand
    find_program_required
    install_cmake_infrastructure
    install_cmake_config_version
    catkin_workspace
    enable_python
    langs
    tools/doxygen
    tools/sphinx
    )
  include(${catkin_EXTRAS_DIR}/${f}.cmake)
endforeach()

if(catkin_BINARY_DIR)
  set(CATKIN_CONTEXT_FILE ${catkin_BINARY_DIR}/catkin-context.py
    CACHE INTERNAL "catkin context file")
  set(CATKIN_ENV ${CMAKE_BINARY_DIR}/env.sh CACHE INTERNAL "catkin env")
  message(STATUS "Shell environment is defined in build directory ${CMAKE_BINARY_DIR}/env.sh")
else()
  set(CATKIN_CONTEXT_FILE ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/catkin-context.py
    CACHE INTERNAL "catkin context file")
  set(CATKIN_ENV ${catkin_INSTALL_PREFIX}/env.sh
    CACHE INTERNAL "catkin env")
  message(STATUS "Shell environment is defined in catkin installation at ${catkin_INSTALL_PREFIX}/env.sh")
endif()
configure_file(${catkin_EXTRAS_DIR}/templates/catkin-context.in ${CATKIN_CONTEXT_FILE})

#
# These get generated no matter what.
#
foreach(shfile setup.zsh setup.bash)
  configure_file(${catkin_EXTRAS_DIR}/templates/${shfile} ${CMAKE_BINARY_DIR}/${shfile}
    @ONLY)
endforeach()

foreach(shfile setup.sh env.sh)
  configure_file(${catkin_EXTRAS_DIR}/templates/${shfile}.buildspace.in ${CMAKE_BINARY_DIR}/${shfile}
    @ONLY)
endforeach()

