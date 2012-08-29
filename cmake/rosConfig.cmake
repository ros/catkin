# only exists for backward compatibility

message(WARNING "find_package(ROS ...) is deprecated, use find_package(catkin ...) instead")

# prevent usage with wrong case
if(ros_FIND_COMPONENTS OR ros_FIND_REQUIRED OR ros_FIND_QUIETLY)
  message(FATAL_ERROR "find_package() only supports upper-case package name 'ROS'")
endif()

  # find package component
if(ROS_FIND_REQUIRED)
  find_package(catkin REQUIRED COMPONENTS ${ROS_FIND_COMPONENTS})
elseif(ROS_FIND_QUIETLY)
  find_package(catkin QUIET COMPONENTS ${ROS_FIND_COMPONENTS})
else()
  find_package(catkin COMPONENTS ${ROS_FIND_COMPONENTS})
endif()

set(ROS_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
set(ROS_LIBRARIES ${catkin_LIBRARIES})
set(ROS_LIBRARY_DIRS ${catkin_LIBRARY_DIRS})
