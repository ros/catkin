# XXX reenable later
set(programs
  catkin_find
  catkin_init_workspace
  catkin_package_version
  catkin_prepare_release
  catkin_test_results
  catkin_topological_order
  catkin_ws

  #catkin-build-debs-of-workspace
)

# install all programs
install(PROGRAMS
  ${programs}
  DESTINATION bin
)

# generate relay-script for each program
foreach(program ${programs})
  set(EXECUTABLE ${catkin_SOURCE_DIR}/bin/${program})
  configure_file(${catkin_EXTRAS_DIR}/templates/script.sh.in
    ${CATKIN_BUILD_PREFIX}/bin/${program}
    @ONLY)
endforeach()
