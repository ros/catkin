# XXX reenable later
set(programs
  catkin_find
  catkin_init_workspace
  catkin_prepare_release
  catkin_stack_version
  catkin_test_results
  catkin_topological_order
  catkin_ws

  #catkin-build-debs-of-workspace
  #catkin-parse-stack
  #catkin_install_parse
  #catkin_util.sh
  #git-catkin
  #git-catkin-track-all
)

# install all programs
install(PROGRAMS
  ${programs}
  DESTINATION ${GLOBAL_BIN_DESTINATION}
)

# generate relay-script for each program
foreach(program ${programs})
  set(EXECUTABLE ${catkin_SOURCE_DIR}/bin/${program})
  configure_file(${catkin_EXTRAS_DIR}/templates/script.sh.in
    ${catkin_BUILD_PREFIX}/${GLOBAL_BIN_DESTINATION}/${program}
    @ONLY)
endforeach()
