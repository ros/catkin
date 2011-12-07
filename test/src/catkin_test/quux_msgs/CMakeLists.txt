cmake_minimum_required(VERSION 2.8)
project(quux_msgs)

find_package(genmsg)
find_package(std_msgs)

add_message_files(
  DIRECTORY msg
  FILES QuuxString.msg
)

generate_messages(DEPENDENCIES std_msgs)

install_cmake_infrastructure(${PROJECT_NAME}
  VERSION 0.0.1
  MSG_DIRS msg
  )
