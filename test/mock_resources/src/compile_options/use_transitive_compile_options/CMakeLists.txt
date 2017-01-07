cmake_minimum_required(VERSION 2.8.12)
project(use_transitive_compile_options)

find_package(catkin REQUIRED COMPONENTS use_compile_options)
add_compile_options(${catkin_COMPILE_OPTIONS})

catkin_package(
  LIBRARIES use_transitive_compile_options_lib
 )
add_library(use_transitive_compile_options_lib lib.cpp)
