if (NOT TARGET gendebian)
  add_custom_target(gendebian)
endif()

if (NOT TARGET gendebian-files)
  add_custom_target(gendebian-files)
endif()

set(CATKIN_DPKG_BUILDPACKAGE_FLAGS "-S" CACHE STRING 
  "Flags passed when running dpkg-buildpackage as part of -gendeiban targets")
  
#EAR: make sure to declare cache variables so that the cmake doesn't warn about unused manually specified cache variables.
option(CATKIN_DEB_SNAPSHOTS "Use a snapshot timestamp in the debian package name." YES)

function(catkin_package PKGNAME)

  stamp(${PROJECT_SOURCE_DIR}/stack.yaml)

  safe_execute_process(COMMAND
    ${catkin_EXTRAS_DIR}/stack_get.py
    ${PROJECT_SOURCE_DIR}/stack.yaml
    ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/stack.cmake
    )

  include(${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/stack.cmake)

  assert(CATKIN_ENV)

  add_custom_target(
    ${PROJECT_NAME}-gendebian-files

    COMMAND
    ${CATKIN_ENV}
    ${catkin_EXTRAS_DIR}/catkin_generate_debian.py
    ${CATKIN_CONTEXT_FILE}
    ${PROJECT_SOURCE_DIR}/stack.yaml  # input
    ${catkin_EXTRAS_DIR}/em           # templates
    ${PROJECT_SOURCE_DIR}/debian      # outdir
    ${CMAKE_BINARY_DIR}

    COMMENT "Generating debian directory *in-source* for stack ${PROJECT_NAME}"
    )

  add_custom_target(
    ${PROJECT_NAME}-gendebian
    )
  add_custom_command(TARGET ${PROJECT_NAME}-gendebian
    COMMAND dpkg-buildpackage ${CATKIN_DPKG_BUILDPACKAGE_FLAGS} # ${PROJECT_SOURCE_DIR}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    )

  catkin_make_dist(${PROJECT_NAME} ${${PKGNAME}_VERSION})
  
  add_custom_target(
    ${PROJECT_NAME}-gbp
    
    COMMAND
    ${CATKIN_ENV}
    ${catkin_EXTRAS_DIR}/catkin_generate_gbp.py
    --repo gbp_${PROJECT_NAME}
    --upstream ${CMAKE_BINARY_DIR}/${PROJECT_NAME}_${${PKGNAME}_VERSION}.orig.tar.gz
    --version ${${PKGNAME}_VERSION}
    --rosdistro electric
    --build_path ${CMAKE_BINARY_DIR}/gbp_${PROJECT_NAME}_build
    --deb_path ${CMAKE_BINARY_DIR}/debs
    
    COMMENT "Generating debs using git-buildpackage"
    
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
  add_dependencies(${PROJECT_NAME}-gbp ${PROJECT_NAME}-dist)
  add_dependencies(${PROJECT_NAME}-gendebian ${PROJECT_NAME}-gendebian-files)
  add_dependencies(gendebian-files ${PROJECT_NAME}-gendebian-files)
  add_dependencies(gendebian ${PROJECT_NAME}-gendebian)

endfunction()

