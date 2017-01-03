cmake_minimum_required(VERSION 2.8.3)
project(use_transitive_definitions)

find_package(catkin REQUIRED COMPONENTS use_definitions)
add_definitions(${catkin_DEFINITIONS})

catkin_package(
  LIBRARIES use_transitive_definitions_lib
 )
add_library(use_transitive_definitions_lib lib.cpp)
