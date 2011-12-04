cmake_minimum_required(VERSION 2.8)
project(catkin_test_nolangs)
find_package(catkin REQUIRED)

add_executable(catkin_test_nolangs_exec
  main.cpp)

install(TARGETS catkin_test_nolangs_exec
  RUNTIME DESTINATION bin)

install_cmake_infrastructure(catkin_test_nolangs
  VERSION 0.0.1
)

catkin_package(catkin_test_nolangs_exec)


