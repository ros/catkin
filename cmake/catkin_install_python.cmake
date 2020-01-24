#
# Install Python files and update their shebang lines
# to use a different Python executable.
#
# The signature:
#
#   catkin_install_python(PROGRAMS files... DESTINATION <dir> [OPTIONAL])
#
# See the documentation for CMake install() function for more information.
#
# @public
#
function(catkin_install_python signature)
  string(TOUPPER "${signature}" signature)
  if(NOT "${signature}" STREQUAL "PROGRAMS")
    message(FATAL_ERROR "catkin_install_python() only supports the PROGRAMS signature (not '${signature}').")
  endif()
  cmake_parse_arguments(ARG "OPTIONAL" "DESTINATION" "" ${ARGN})
  if(NOT ARG_DESTINATION)
    message(FATAL_ERROR "catkin_install_python() called without required DESTINATION argument.")
  endif()
  foreach(source_file ${ARG_UNPARSED_ARGUMENTS})
    if(NOT IS_ABSOLUTE ${source_file})
      set(source_file "${CMAKE_CURRENT_SOURCE_DIR}/${source_file}")
    endif()
    if(EXISTS ${source_file})
      stamp(${source_file})
      # read file and check shebang line
      file(READ ${source_file} data)
      set(regex "^#!/([^\r\n]+)/env python([\r\n])")
      string(REGEX MATCH "${regex}" shebang_line "${data}")
      string(LENGTH "${shebang_line}" length)
      string(SUBSTRING "${data}" 0 ${length} prefix)
      if("${shebang_line}" STREQUAL "${prefix}")
        # write modified file with modified shebang line
        get_filename_component(python_name ${PYTHON_EXECUTABLE} NAME)
        string(REGEX REPLACE "${regex}" "#!/\\1/env ${python_name}\\2" data "${data}")
        get_filename_component(filename ${source_file} NAME)
        set(rewritten_file "${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace")
        file(MAKE_DIRECTORY ${rewritten_file})
        set(rewritten_file "${rewritten_file}/${filename}")
        file(WRITE ${rewritten_file} "${data}")
      else()
        # Shebang did not match, install file unmodified
        set(rewritten_file "${source_file}")
      endif()
      # install (modified) file to destination
      set(optional_flag "")
      if(ARG_OPTIONAL)
        set(optional_flag "OPTIONAL")
      endif()
      # Install copy of file with re-written shebang to install space
      install(PROGRAMS "${rewritten_file}" DESTINATION "${ARG_DESTINATION}" ${optional_flag})

      # Hook for a platform specific wrapper around the modified python script
      get_filename_component(name "${rewritten_file}" NAME)
      add_python_executable(SCRIPT_NAME ${name}
        # prefix with project name to avoid collisions across packages
        TARGET_NAME ${PROJECT_NAME}_${name}_exec_install_python
        DESTINATION "${ARG_DESTINATION}")
    elseif(NOT ARG_OPTIONAL)
      message(FATAL_ERROR "catkin_install_python() called with non-existing file '${source_file}'.")
    endif()
  endforeach()
endfunction()
