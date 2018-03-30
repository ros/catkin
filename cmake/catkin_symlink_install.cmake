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

option(CATKIN_SYMLINK_INSTALL
  "Replace the CMake install command with a custom implementation using symlinks instead of copying resources"
  OFF)

if(CATKIN_SYMLINK_INSTALL)
  message(STATUS "Override CMake install command with custom implementation "
    "using symlinks instead of copying resources")

  include(
    "${catkin_EXTRAS_DIR}/symlink_install/catkin_symlink_install_append_install_code.cmake")
  include(
    "${catkin_EXTRAS_DIR}/symlink_install/catkin_symlink_install_directory.cmake")
  include(
    "${catkin_EXTRAS_DIR}/symlink_install/catkin_symlink_install_files.cmake")
  include(
    "${catkin_EXTRAS_DIR}/symlink_install/catkin_symlink_install_programs.cmake")
  include(
    "${catkin_EXTRAS_DIR}/symlink_install/catkin_symlink_install_targets.cmake")
  include("${catkin_EXTRAS_DIR}/symlink_install/catkin_install_logic.cmake")

  # register custom install logic
  _use_custom_install()
  set(_CATKIN_CUSTOM_INSTALL_RULES "${catkin_EXTRAS_DIR}/symlink_install/install.cmake")

  # create the install script from the template
  # catkin/cmake/symlink_install/catkin_symlink_install.cmake.in
  set(CATKIN_SYMLINK_INSTALL_INSTALL_SCRIPT
  "${CMAKE_CURRENT_BINARY_DIR}/catkin_symlink_install/catkin_symlink_install.cmake")
  configure_file(
    "${catkin_EXTRAS_DIR}/symlink_install/catkin_symlink_install.cmake.in"
    "${CATKIN_SYMLINK_INSTALL_INSTALL_SCRIPT}"
    @ONLY
  )
  # register script for being executed at install time
  install(SCRIPT "${CATKIN_SYMLINK_INSTALL_INSTALL_SCRIPT}")
endif()
