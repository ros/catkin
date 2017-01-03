cmake_minimum_required(VERSION 2.8.3)
project(set_definitions)
find_package(catkin REQUIRED)

catkin_package(
  CFG_EXTRAS set_definitions-extra.cmake.in
  )
