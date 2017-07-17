cmake_minimum_required(VERSION 2.8.12)
project(set_compile_options)
find_package(catkin REQUIRED)

list(APPEND ${PROJECT_NAME}_COMPILE_OPTIONS -DTHIS_IS_MY_EXPORTED_DEFINITION)

add_compile_options(${catkin_COMPILE_OPTIONS} ${${PROJECT_NAME}_COMPILE_OPTIONS})

catkin_package(
  COMPILE_OPTIONS -DTHIS_IS_MY_OTHER_EXPORTED_DEFINITION=1
  )
