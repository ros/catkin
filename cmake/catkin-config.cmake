#
# catkin-config.cmake
#
set(catkin_EXTRAS_DIR /home/overhaul/src/catkin_test/nose/src/catkin/cmake)
include(${catkin_EXTRAS_DIR}/all.cmake)


if (NOT catkin_FOUND)
  #set(CATKIN_PACKAGE_PREFIX "")
  #set(CATKIN_LINUX_DISTRIBUTIONS "lucid maverick natty oneiric")
  set(catkin_FOUND)
  
  set(CATKIN_CONTEXT_FILE ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/catkin-context.py)

  configure_file(${catkin_EXTRAS_DIR}/catkin-context.in ${CATKIN_CONTEXT_FILE})

  set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH}:${CMAKE_BINARY_DIR}/cmake)

endif()
