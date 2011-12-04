if (NOT TARGET gendebian)
  add_custom_target(gendebian)
endif()

if (NOT TARGET gendebian-files)
  add_custom_target(gendebian-files)
endif()

set(CATKIN_DPKG_BUILDPACKAGE_FLAGS "-S" CACHE STRING 
  "Flags passed when running dpkg-buildpackage as part of -gendeiban targets")

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
  add_dependencies( ${PROJECT_NAME}-gendebian ${PROJECT_NAME}-gendebian-files)
  add_dependencies(gendebian-files ${PROJECT_NAME}-gendebian-files)
  add_dependencies(gendebian ${PROJECT_NAME}-gendebian)

endfunction()

