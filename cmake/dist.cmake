if (NOT TARGET dist)
  add_custom_target(dist)
endif()


macro( catkin_make_dist project version)
    add_custom_target(${project}-dist
      COMMENT "Generating an upstream tarball --- ${CMAKE_BINARY_DIR}/${project}_${version}.orig.tar.gz"
    )
    
    #just uses tar to create an upstream tarball
    #excludes common vcs dirs
    add_custom_command(TARGET ${project}-dist
      COMMAND
      tar --exclude-vcs
       -czf
      ${CMAKE_BINARY_DIR}/${project}_${version}.orig.tar.gz
      ./
      WORKING_DIRECTORY ${${project}_SOURCE_DIR}
      )
    add_dependencies(dist ${project}-dist)
endmacro()

