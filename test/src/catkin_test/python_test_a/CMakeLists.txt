cmake_minimum_required(VERSION 2.8)
project(python_test_a)

find_package(genmsg)
find_package(std_msgs)

add_message_files(
  DIRECTORY msg
  FILES A.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_project(${PROJECT_NAME}
  VERSION 0.0.1
  )
