
# Path separation is handled differently on win/unix platforms.
if(CMAKE_HOST_WIN32)
  set(CATKIN_PATH_SEPARATOR ";")
else()
  set(CATKIN_PATH_SEPARATOR ":")
endif()
