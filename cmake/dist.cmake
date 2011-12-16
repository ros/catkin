if (NOT TARGET dist)
  add_custom_target(dist)
endif()


macro( catkin_make_dist project version)
    set(${project}_tarball_name ${CMAKE_BINARY_DIR}/${project}-${version}.tar.gz) 
    add_custom_target(${project}-dist
      COMMENT "Generating an upstream tarball --- ${${project}_tarball_name}"
    )
    
    #just uses tar to create an upstream tarball
    #excludes common vcs dirs
    add_custom_command(TARGET ${project}-dist
      COMMAND
      tar --exclude-vcs
       -czf
      ${${project}_tarball_name}
      ./
      WORKING_DIRECTORY ${${project}_SOURCE_DIR}
      )
    add_dependencies(dist ${project}-dist)
endmacro()

