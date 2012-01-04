#
#  used to thunk out to catkin, from rosbuild cmakelists files.
#  For saving time during transition.
#

#EAR: setting ENV is frowned upon, this has global scope ...
#please use the env.sh script if you require ROS_ROOT to be set when running scripts
#set(ENV{ROS_ROOT} ${CMAKE_SOURCE_DIR}/ros)

cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0011 NEW)

macro(rosbuild_catkinize)
  if(CATKIN)
    cmake_policy(SET CMP0003 NEW)
    cmake_policy(SET CMP0011 NEW)
    message(STATUS "    >> Rosbuild-compat: thunking from rosbuild in ${CMAKE_CURRENT_SOURCE_DIR}")
    if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/catkin.cmake)
      include(catkin.cmake NO_POLICY_SCOPE)
    else()
      message(STATUS "    >> warning: Directory ${CMAKE_CURRENT_SOURCE_DIR} contains rosbuild_catkinize but no catkin.cmake")
    endif()
    return()
  endif()
endmacro()


function(rosbuild_add_executable TARGET)
  message(STATUS "    >> Rosbuild-compat: rosbuild_add_executable ${ARGV}")
  add_executable(${TARGET} ${ARGN})
endfunction()

function(rosbuild_add_library TARGET)
  message(STATUS "    >> Rosbuild-compat: rosbuild_add_library ${ARGV}")
  add_library(${TARGET} SHARED ${ARGN})
endfunction()


function(rosbuild_link_boost TARGET)
  message(STATUS "    >> Rosbuild-compat link boost ${ARGN}")
endfunction()

function(rosbuild_download_test_data)
  message(STATUS "    >> Rosbuild-compat: rosbuild_download_test_data ${ARGN}")
endfunction()

function(rosbuild_add_pyunit)
  message(STATUS "    >> Rosbuild-compat: rosbuild_add_pyunit ${ARGN}")
endfunction()

function(rosbuild_add_gtest TARGET)
  message(STATUS "    >> Rosbuild-compat: rosbuild_add_gtest ${ARGN}")
  add_executable(${TARGET} ${catkin_EXTRAS_DIR}/dummy_main.cpp)
  set_property(TARGET ${TARGET}
    PROPERTY
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin
    )
endfunction()

function(rosbuild_add_rostest TARGET)
  message(STATUS "    >> Rosbuild-compat: rosbuild_add_rostest ${TARGET}")
  string(REPLACE "/" "_" _testname ${TARGET})

  add_executable(rostest_${_testname} ${catkin_EXTRAS_DIR}/dummy_main.cpp)
  set_property(TARGET rostest_${_testname}
    PROPERTY
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin
    )
endfunction()

