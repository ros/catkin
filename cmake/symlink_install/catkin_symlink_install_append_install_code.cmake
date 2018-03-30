# Copyright 2014 Open Source Robotics Foundation, Inc.
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
# Register a CMake script for execution at install time.
#
# :param ARGN: the list of CMake code lines
# :type ARGN: list of strings
# :param COMMENTS: an optional list of comments
# :type COMMENTS: list of strings
#
function(catkin_symlink_install_append_install_code)
  cmake_parse_arguments(ARG "" "" "COMMENTS" ${ARGN})

  # append code to install script
  if(ARG_COMMENTS)
    file(APPEND "${CATKIN_SYMLINK_INSTALL_INSTALL_SCRIPT}"
      "\n# ${ARG_COMMENTS}\n")
  endif()
  foreach(code ${ARG_UNPARSED_ARGUMENTS})
    file(APPEND "${CATKIN_SYMLINK_INSTALL_INSTALL_SCRIPT}" "${code}\n")
  endforeach()
endfunction()
