
execute_process(COMMAND date +%F-%0k-%0M-%0S%z
  OUTPUT_VARIABLE DEBIAN_SNAPSHOT_SUFFIX
  OUTPUT_STRIP_TRAILING_WHITESPACE)

if (NOT DEBS_TARGET_ADDED)
  add_custom_target(debs)
  set(DEBS_TARGET_ADDED TRUE CACHE INTERNAL "onetime")
endif()

function(catkin_package PKGNAME)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/debian/control
    ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/debian/control.stamp
    @ONLY
    )

  # see http://www.mail-archive.com/cmake@cmake.org/msg03461.html
  add_custom_target(${PKGNAME}-install
    COMMAND ${CMAKE_COMMAND} -DCOMPONENT=${PACKAGE_NAME} -P cmake_install.cmake
    WORKING_DIRECTORY ${${PACKAGE_NAME}_BINARY_DIR}
    COMMENT "making binary deb for package ${PKGNAME}"
    )

  log(1 "${PROJECT_NAME}: Enabling deb target since directory 'debian' exists")
  safe_execute_process(COMMAND /bin/mkdir -p ${CMAKE_BINARY_DIR}/debs)
  add_custom_target(${PROJECT_NAME}-dsc
    COMMAND dpkg-source -b ${CMAKE_CURRENT_SOURCE_DIR}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/debs
    )
  add_dependencies(debs ${PROJECT_NAME}-deb)

endfunction()
