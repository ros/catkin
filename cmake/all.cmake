include(${catkin_EXTRAS_DIR}/log.cmake)
include(${catkin_EXTRAS_DIR}/assert.cmake)
assert(catkin_EXTRAS_DIR)

foreach(f
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
    )
  include(${catkin_EXTRAS_DIR}/${f}.cmake)
endforeach()

if(catkin_BINARY_DIR)
  set(CATKIN_CONTEXT_FILE ${catkin_BINARY_DIR}/catkin-context.py
    CACHE INTERNAL "catkin context file")
  set(CATKIN_ENV ${CMAKE_BINARY_DIR}/env.sh)
else()
  set(CATKIN_CONTEXT_FILE ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/catkin-context.py
    CACHE INTERNAL "catkin context file")
  set(CATKIN_ENV ${catkin_INSTALL_PREFIX}/env.sh)
endif()
configure_file(${catkin_EXTRAS_DIR}/catkin-context.in ${CATKIN_CONTEXT_FILE})

