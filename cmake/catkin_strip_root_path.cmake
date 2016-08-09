#
# Strip CMAKE_FIND_ROOT_PATH prefixes from each path in the list ``var``.
#
# .. note:: Used for cross-compilation.
#
# :param var: the list to strip
# :type var: list of paths
# :param UNIQUE: if specified the stripped list will contain unique paths, i.e.,
#   duplicate paths are not appended a second time
# :type UNIQUE: option
#
function(catkin_strip_root_path var)
  cmake_parse_arguments(ARG "UNIQUE" "" "" ${ARGN})
  set(_list)
  foreach(_ele ${${var}})
    if(IS_ABSOLUTE ${_ele})
      foreach(_path ${CMAKE_FIND_ROOT_PATH})
        string(REGEX REPLACE "^${_path}/" "/" _ele ${_ele})
      endforeach()
    endif()
    if(ARG_UNIQUE)
      list_append_unique(_list ${_ele})
    else()
      list(APPEND _list ${_ele})
    endif()
  endforeach()
  set(${var} ${_list} PARENT_SCOPE)
endfunction()
