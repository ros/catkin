cmake_minimum_required(VERSION 2.8)
project(catkin_test_nolangs)
find_package(catkin REQUIRED)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)

add_executable(catkin_test_nolangs_exec
  main.cpp)

install(TARGETS catkin_test_nolangs_exec
  RUNTIME DESTINATION bin)

catkin_project(catkin_test_nolangs
  VERSION 0.0.1
)

catkin_stack()


