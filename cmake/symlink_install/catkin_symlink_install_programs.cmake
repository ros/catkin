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
# Reimplement CMake install(PROGRAMS) command to use symlinks instead of copying
# resources.
#
# :param ARGN: the same arguments as the CMake install command.
# :type ARGN: various
#
function(catkin_symlink_install_programs programs_keyword)
  if(NOT programs_keyword STREQUAL "PROGRAMS")
    message(FATAL_ERROR "catkin_symlink_install_programs() first argument "
      "must be 'PROGRAMS', not '${programs_keyword}'")
  endif()

  set(unsupported_keywords
    "PERMISSIONS"
    "CONFIGURATIONS"
    "COMPONENT"
    "RENAME"
  )
  foreach(unsupported_keyword ${unsupported_keywords})
    list(FIND ARGN "${unsupported_keyword}" index)
    if(NOT index EQUAL -1)
      # fall back to CMake install() command
      # if the arguments can't be handled
      _install(PROGRAMS ${ARGN})
      break()
    endif()
  endforeach()

  if(index EQUAL -1)
    string(REPLACE ";" "\" \"" argn_quoted "\"${ARGN}\"")
    catkin_symlink_install_append_install_code(
      "catkin_symlink_install_programs(PROGRAMS ${argn_quoted})"
      COMMENTS "install(PROGRAMS ${argn_quoted})"
    )
  endif()
endfunction()
