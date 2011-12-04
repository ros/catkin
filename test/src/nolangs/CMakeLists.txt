cmake_minimum_required(VERSION 2.8)
project(nolangs)
find_package(catkin REQUIRED)

add_executable(nolangs_exec
  main.cpp)

install(TARGETS nolangs_exec
  RUNTIME DESTINATION bin)

install_cmake_infrastructure(nolangs
  VERSION 0.0.1
)

catkin_package(nolangs)


