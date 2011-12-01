execute_process(COMMAND date +%F-%0k-%0M-%0S%z
  OUTPUT_VARIABLE DEBIAN_SNAPSHOT_SUFFIX
  OUTPUT_STRIP_TRAILING_WHITESPACE)

if (NOT TARGET debs)
  add_custom_target(debs)
endif()

function(catkin_package PKGNAME)

  stamp(${PROJECT_SOURCE_DIR}/stack.yaml)

  safe_execute_process(COMMAND
    ${catkin_EXTRAS_DIR}/stack_get.py
    ${PROJECT_SOURCE_DIR}/stack.yaml
    ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/stack.cmake
    )

  include(${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/stack.cmake)

  if (NOT CATKIN_ENABLE_DEBBUILDING)
    return()
  endif()

  assert(CATKIN_ENV)
  message("CATKIN_ENV IS NOW ${CATKIN_ENV}")
  #
  # Generate control file
  #
  add_custom_command(

    OUTPUT
    ${PROJECT_SOURCE_DIR}/debian/control
    ${PROJECT_SOURCE_DIR}/debian/rules
    ${PROJECT_SOURCE_DIR}/debian/changelog
    ${PROJECT_SOURCE_DIR}/debian/copyright

    COMMAND pwd
    COMMAND
    ${CATKIN_ENV}
    ${catkin_EXTRAS_DIR}/catkin_generate_debian.py
    ${CATKIN_CONTEXT_FILE}
    ${PROJECT_SOURCE_DIR}/stack.yaml  # input
    ${catkin_EXTRAS_DIR}/em           # templates
    ${PROJECT_SOURCE_DIR}/debian      # outdir
    ${CMAKE_BINARY_DIR}

    DEPENDS
    ${PROJECT_SOURCE_DIR}/stack.yaml
    ${catkin_EXTRAS_DIR}/catkin_generate_debian.py
    ${catkin_EXTRAS_DIR}/em/control.em
    ${catkin_EXTRAS_DIR}/em/rules.cmake.em
    ${catkin_EXTRAS_DIR}/em/copyright.willowgarage.em
    ${catkin_EXTRAS_DIR}/em/changelog.em

    COMMENT "Shelling out..."
    VERBATIM
    )

  add_custom_target(
    ${PROJECT_NAME}-gendebian

    DEPENDS
    ${PROJECT_SOURCE_DIR}/debian/control
    ${PROJECT_SOURCE_DIR}/debian/rules
    ${PROJECT_SOURCE_DIR}/debian/changelog
    ${PROJECT_SOURCE_DIR}/debian/copyright

    COMMENT "Generating debian directory *in-source* for stack ${PROJECT_NAME}"
    )

  add_custom_command(TARGET ${PROJECT_NAME}-gendebian
    POST_BUILD
    COMMAND dpkg-source -b ${PROJECT_SOURCE_DIR}
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
    )

endfunction()

