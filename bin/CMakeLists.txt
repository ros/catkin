# TODO: consider installing em somewhere else
install(DIRECTORY em/
  DESTINATION bin/em
  USE_SOURCE_PERMISSIONS
  )
install(PROGRAMS
  catkin-build-debs-of-workspace
  catkin-bump-version
  catkin-generate-debian
  catkin-parse-stack
  catkin-topological-order
  catkin-version
  catkin_install_parse
  catkin_util.sh
  git-catkin
  git-catkin-generate-debian
  git-catkin-import-upstream
  git-catkin-set-upstream
  git-catkin-track-all
  DESTINATION bin
)
