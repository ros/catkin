if (NOT catkin_SOURCE_DIR)
  if (NOT CATKIN_GENLANGS)
    file(GLOB CATKIN_GENLANGS
      RELATIVE ${CMAKE_INSTALL_PREFIX}/etc/langs
      ${CMAKE_INSTALL_PREFIX}/etc/langs/gen*)
    message(STATUS "Using these generator languages from the installation: ${CATKIN_GENLANGS}")
  endif()
endif()