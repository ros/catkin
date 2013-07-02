cmake_minimum_required(VERSION 2.8.3)

set(catkin_EXTRAS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

project(catkin)

if(NOT DEFINED CATKIN_TOPLEVEL)
  include(${catkin_EXTRAS_DIR}/all.cmake NO_POLICY_SCOPE)
endif()

catkin_package()

install(FILES LICENSE
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# for convenience we provide eigen-config(-version).cmake files
file(COPY cmake/Modules/eigen-config.cmake cmake/Modules/eigen-config-version.cmake
  DESTINATION ${CATKIN_DEVEL_PREFIX}/share/eigen/cmake
)
install(FILES cmake/Modules/eigen-config.cmake cmake/Modules/eigen-config-version.cmake
  DESTINATION share/eigen/cmake
)

# install catkin scripts
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/bin)

# install cmake files for catkin (except some files only used at build time of catkin)
install(DIRECTORY cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN "catkinConfig.cmake.in" EXCLUDE
  PATTERN "Modules" EXCLUDE)

catkin_python_setup()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/local_tests)
  catkin_add_nosetests(test/unit_tests)
endif()
