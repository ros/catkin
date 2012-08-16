# macro to ensure that ${PROJECT_NAME}_CACHE gets set in a higher scope
# where we can check it later
macro(append_test_to_cache CACHENAME)
  set(cachefile ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}.${CACHENAME})
  if(NOT ${PROJECT_NAME}_CACHE)
    file(WRITE ${cachefile} "#\n# This is ${cachefile}\n#\n")
    # write out the directory that we'll use as CWD later when running tests
    # requires newline at the end so that the message appears in the cache
    file(APPEND ${cachefile} "${CMAKE_CURRENT_SOURCE_DIR}\n")
    set(${PROJECT_NAME}_CACHE TRUE)
    set(${PROJECT_NAME}_CACHE TRUE PARENT_SCOPE)
    # one test target per project
    add_custom_target(${PROJECT_NAME}_run_tests
      COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE}
      ${catkin_EXTRAS_DIR}/test/runtests.py ${cachefile})
    add_dependencies(test ${PROJECT_NAME}_run_tests)
    add_dependencies(${PROJECT_NAME}_run_tests tests)
  endif()

  # requires newline at the end so that the message appears in the cache
  file(APPEND ${cachefile} "${ARGN}\n")
endmacro()
