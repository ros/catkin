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
# Reimplement CMake install(DIRECTORY) command to use symlinks instead of
# copying resources.
#
# :param ARGN: the same arguments as the CMake install command.
# :type ARGN: various
#
function(catkin_symlink_install_directory directory_keyword)
  if(NOT directory_keyword STREQUAL "DIRECTORY")
    message(FATAL_ERROR "catkin_symlink_install_directory() first "
      "argument must be 'DIRECTORY', not '${directory_keyword}'")
  endif()

  set(unsupported_keywords
    "FILE_PERMISSIONS"
    "DIRECTORY_PERMISSIONS"
    "USE_SOURCE_PERMISSIONS"
    "CONFIGURATIONS"
    "COMPONENT"
    "FILES_MATCHING"
    "REGEX"
    "PERMISSIONS"
  )
  foreach(unsupported_keyword ${unsupported_keywords})
    list(FIND ARGN "${unsupported_keyword}" index)
    if(NOT index EQUAL -1)
      # fall back to CMake install() command
      # if the arguments can't be handled
      _install(DIRECTORY ${ARGN})
      break()
    endif()
  endforeach()

  if(index EQUAL -1)
    # merge 'PATTERN "xxx" EXCLUDE' arguments to 'PATTERN_EXCLUDE "xxx"'
    set(argn ${ARGN})
    list(LENGTH argn length)
    set(i 0)
    while(i LESS length)
      list(GET argn ${i} arg)
      if(arg STREQUAL "PATTERN")
        math(EXPR j "${i} + 2")
        if(j LESS length)
          list(GET argn ${j} arg)
          if(arg STREQUAL "EXCLUDE")
            # replace "PATTERN" with "PATTERN_EXCLUDE"
            list(REMOVE_AT argn ${i})
            list(INSERT argn ${i} "PATTERN_EXCLUDE")
            # remove "EXCLUDE"
            list(REMOVE_AT argn ${j})
            # get changed length
            list(LENGTH argn length)
          endif()
        endif()
      endif()
      math(EXPR i "${i} + 1")
    endwhile()

    string(REPLACE ";" "\" \"" argn_quoted "\"${argn}\"")
    catkin_symlink_install_append_install_code(
      "catkin_symlink_install_directory(DIRECTORY ${argn_quoted})"
      COMMENTS "install(DIRECTORY ${argn_quoted})"
    )
  endif()
endfunction()
