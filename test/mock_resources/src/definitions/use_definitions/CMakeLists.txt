cmake_minimum_required(VERSION 2.8.3)
project(use_definitions)

find_package(catkin REQUIRED COMPONENTS set_definitions)
add_definitions(${catkin_DEFINITIONS})

catkin_package(
  LIBRARIES use_definitions_lib
  CATKIN_DEPENDS set_definitions
 )
add_library(use_definitions_lib lib.cpp)
