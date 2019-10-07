cmake_minimum_required(VERSION 3.0.2)
project(leaf_pkg)

find_package(catkin REQUIRED COMPONENTS root_pkg)

add_executable(leaf leaf.cpp)
target_link_libraries(leaf ${catkin_LIBRARIES})
target_include_directories(leaf PUBLIC ${catkin_INCLUDE_DIRS})

catkin_package()

list(LENGTH catkin_LIBRARIES size)
math(EXPR size_is_not_3 "${size} - 3")
if (size_is_not_3)
  message(FATAL_ERROR "Wrong size for catkin_LIBRARIES: ${catkin_LIBRARIES} : ${size}")
else()
  message(STATUS "All is OK: ${catkin_LIBRARIES}")
endif()
