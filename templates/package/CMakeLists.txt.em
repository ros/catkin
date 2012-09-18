@# This is a template for a CMakeLists.txt in a catkin package
@# See empy -H for the templating syntax
@{
import em
}@
cmake_minimum_required(VERSION 2.8)
project(@empy.CATKIN_PACKAGE.name)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED @{
if empy.CATKIN_PACKAGE.components is not None and len(empy.CATKIN_PACKAGE.components) > 0:
  print('COMPONENTS\n  ' + '\n  '.join(sorted(empy.CATKIN_PACKAGE.components)))})

## Uncomment this if this package is also its own stack (unary stack)
@{
if empy.CATKIN_PACKAGE.unary:
  print('catkin_stack()')
else:
  print('# catkin_stack()')
}

catkin_project(${PROJECT_NAME}
#  LIBRARIES ${PROJECT_NAME}
#  INCLUDE_DIRS include
#  DEPENDS 
  )


## uncomment if the package has a setup.py
# catkin_python_setup()


## Declare ROS messages and services here

# add_message_files(
#   DIRECTORY msg
#   FILES
#     MyMessage1.msg
#     MyMessage2.msg
# )

# add_service_files(
#   DIRECTORY srv
#   FILES
#     MyService1.srv
#     MyService2.srv
# )

## Uncomment this if you have declared any messages or services
## and specify dependencies if any
# generate_messages(
#   DEPENDENCIES 
#   std_msgs)


## System dependencies, Boost as an example, can be used like this
# find_package(Boost REQUIRED COMPONENTS system)
# target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})


## Use this to declare libraries that this package creates
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/main.cpp
#   )

## Use this to declare executables that this package creates
# add_executable(${PROJECT_NAME}_node src/main.cpp)
# target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${Boost_LIBRARIES})


## Use this to e.g. make your executables depend on your libraries
# add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME})

## Specify additional locations of header files
# include_directories(include)

## specify what c++ libraries your c++ executable should be linked against
# target_link_libraries(${PROJECT_NAME}_node
#   ${Boost_LIBRARIES}
#   )


## Use this to declare executable scripts (Python etc.)
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PROJECT_BIN_DESTINATION})


## Use this to declare c++ executables and/or libraries
# install(TARGETS ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PROJECT_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PROJECT_LIB_DESTINATION})

## If specific folders are to be installed (no executable or library), use this
# install(DIRECTORY include/
#   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE)

## This is for any other files that should go into an install
# install(FILES 
#   myfile1
#   myfile2
#   DESTINATION ${CATKIN_PROJECT_INCLUDE_DESTINATION}
# )




## Use this for c++ gtest unit tests
# catkin_add_gtest(${PROJECT_NAME}-test test/mytest)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test_version)
# endif()

## Use this to declare python unit test folders
# catkin_add_nosetests(test)