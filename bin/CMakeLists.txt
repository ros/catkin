# XXX reenable later
set(programs
  catkin-init-workspace
  catkin-topological-order
  #catkin-build-debs-of-workspace
  #catkin-bump-version
  #catkin-parse-stack
  #catkin-version
  #catkin_install_parse
  #catkin_util.sh
  #git-catkin
  #git-catkin-track-all
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
    ${catkin_BUILD_PREFIX}/bin/${program}
    @ONLY)
endforeach()
