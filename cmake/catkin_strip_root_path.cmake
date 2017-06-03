#
# Strip CMAKE_FIND_ROOT_PATH prefixes from each path in the list ``var``.
#
# .. note:: Used for cross-compilation.
#
# :param var: the input / output variable name
# :type var: string
# :param UNIQUE: if specified the stripped list will contain unique paths, i.e.,
#   duplicate paths are not appended a second time
# :type UNIQUE: option
#
function(catkin_strip_root_path var)
  cmake_parse_arguments(ARG "UNIQUE" "" "" ${ARGN})
  set(output)
  foreach(element ${${var}})
    if(IS_ABSOLUTE ${element})
      foreach(root_path ${CMAKE_FIND_ROOT_PATH})
        if(element MATCHES "^${root_path}/")
          string(LENGTH ${root_path} root_path_length)
          string(SUBSTRING ${element} ${root_path_length} -1 element)
          break()
        endif()
      endforeach()
    endif()
    if(ARG_UNIQUE)
      list_append_unique(output ${element})
    else()
      list(APPEND output ${element})
    endif()
  endforeach()
  set(${var} ${output} PARENT_SCOPE)
endfunction()
