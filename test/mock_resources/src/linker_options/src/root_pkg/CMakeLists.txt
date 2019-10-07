cmake_minimum_required(VERSION 3.0.2)
project(root_pkg)

find_package(catkin REQUIRED)

add_library(rootlib SHARED root.cpp)
target_include_directories(rootlib PUBLIC include)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES -pthread -lpthread rootlib
  )

install(TARGETS rootlib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
