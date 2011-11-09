
execute_process(COMMAND date +%F-%0k-%0M-%0S%z
  OUTPUT_VARIABLE DEBIAN_SNAPSHOT_SUFFIX
  OUTPUT_STRIP_TRAILING_WHITESPACE)

function(catkin_package PKGNAME)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/debian/control
    ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/debian/control.stamp
    @ONLY
    )
  add_custom_target(${PKGNAME}-deb
    COMMAND /bin/echo "making deb"
    )
endfunction()
