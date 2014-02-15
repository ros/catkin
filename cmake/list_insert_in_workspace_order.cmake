#
# Insert elements to a list in the same order as the chained catkin workspaces.
#
set(CATKIN_ORDERED_SPACES "")
foreach(_space ${CATKIN_DEVEL_PREFIX} ${CATKIN_WORKSPACES})
  list(APPEND CATKIN_ORDERED_SPACES ${_space})
  if(NOT EXISTS "${_space}/.catkin")
    message(FATAL_ERROR "The path '${_space}' is in CATKIN_WORKSPACES but does not have a .catkin file")
  endif()
  # prepend to existing list of sourcespaces
  file(READ "${_space}/.catkin" _sourcespaces)
  list(APPEND CATKIN_ORDERED_SPACES ${_sourcespaces})
endforeach()

# CATKIN_ORDERED_SPACES_END is the last index in the workspaces list.
list(LENGTH CATKIN_ORDERED_SPACES CATKIN_ORDERED_SPACES_END)
math(EXPR CATKIN_ORDERED_SPACES_END "${CATKIN_ORDERED_SPACES_END} - 1")

debug_message(10 "CATKIN_ORDERED_SPACES ${CATKIN_ORDERED_SPACES}")

function(list_insert_in_workspace_order listname)
  if("${ARGN}" STREQUAL "")
    set(${listname} "" PARENT_SCOPE)
    return()
  endif()

  if("${CATKIN_ORDERED_SPACES}" STREQUAL "")
    set(${listname} ${ARGN} PARENT_SCOPE)
    return()
  endif()

  # Init list of empty slots, one for each workspace
  foreach(i RANGE ${CATKIN_ORDERED_SPACES_END})
    set(slot_${i} "")
  endforeach()
  set(slot_unknown "")

  # Sort the paths into the first matching workspace
  foreach(path ${ARGN})
    string(LENGTH "${path}" path_len)
    set(found OFF)

    foreach(i RANGE ${CATKIN_ORDERED_SPACES_END})
      list(GET CATKIN_ORDERED_SPACES ${i} space)

      # If the path is equal to the workspace, it matches.
      if("${space}" STREQUAL "${path}")
        list(APPEND slot_${i} "${path}")
        set(found ON)
        break()
      endif()

      # If the path is long enough, compare the prefix
      string(LENGTH "${space}" len)
      math(EXPR len "${len} + 1")

      if(${len} LESS ${path_len})
        string(SUBSTRING "${path}" 0 ${len} path_prefix)
        if("${space}/" STREQUAL "${path_prefix}")
          list(APPEND slot_${i} "${path}")
          set(found ON)
          break()
        endif()
      endif()
    endforeach()

    if(NOT found)
      list(APPEND slot_unknown "${path}")
    endif()
  endforeach()

  # Flatten the slot list
  set(output "")
  foreach(i RANGE ${CATKIN_ORDERED_SPACES_END})
    list(APPEND output ${slot_${i}})
  endforeach()
  list(APPEND output ${slot_unknown})

  set(${listname} ${output} PARENT_SCOPE)
endfunction()
