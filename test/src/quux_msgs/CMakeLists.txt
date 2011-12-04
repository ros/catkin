cmake_minimum_required(VERSION 2.8)
project(quux_msgs)

find_package(genmsg)
find_package(std_msgs)

generate_msgs(${PROJECT_NAME}
  MESSAGES msg/QuuxString.msg
  DEPENDENCIES std_msgs
  )

install_cmake_infrastructure(${PROJECT_NAME}
  VERSION 0.0.1
  MSG_DIRS msg
  )
