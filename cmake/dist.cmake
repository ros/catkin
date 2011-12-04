macro( catkin_make_dist project version)
add_custom_target(${project}-dist)
##TODO detect version control... for now assume git
add_custom_command(TARGET ${project}-dist
  COMMAND git archive master | gzip > ${CMAKE_BINARY_DIR}/${project}_${version}.orig.tar.gz
  WORKING_DIRECTORY ${${project}_SOURCE_DIR}
  )
endmacro()

