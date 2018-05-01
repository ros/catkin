# Copyright 2014-2018 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Custom CMake install logic to use symlinks instead of copying resources.
#
# :param ARGN: the same arguments as the CMake install command.
# :type ARGN: various
#
function(catkin_install_logic signature)
  string(TOUPPER "${signature}" signature)

  if(signature STREQUAL "DIRECTORY")
    catkin_symlink_install_directory(DIRECTORY ${ARGN})
    return()
  elseif(signature STREQUAL "FILES")
    catkin_symlink_install_files(FILES ${ARGN})
    return()
  elseif(signature STREQUAL "PROGRAMS")
    catkin_symlink_install_programs(PROGRAMS ${ARGN})
    return()
  elseif(signature STREQUAL "TARGETS")
    catkin_symlink_install_targets(TARGETS ${ARGN})
    return()
  endif()

  # fall back to CMake install() command
  # if the arguments haven't been handled before
  _install(${signature} ${ARGN})
endfunction()
