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

# CATKIN_ORDERED_SPACES_END is the last index in the workspaces list,
# which includes an empty prefix at the end for paths which do not belong
# in a known workspace.
list(LENGTH CATKIN_ORDERED_SPACES CATKIN_ORDERED_SPACES_END)
list(APPEND CATKIN_ORDERED_SPACES "")

debug_message(10 "CATKIN_ORDERED_SPACES ${CATKIN_ORDERED_SPACES}")

macro(list_insert_in_workspace_order listname)
  if(NOT "${ARGN}" STREQUAL "")
    # Init list of empty slots, one for each workspace and one with an
    # empty prefix
    foreach(i RANGE ${CATKIN_ORDERED_SPACES_END})
      set(slot_${i} "")
    endforeach()

    # Sort the paths into the first matching workspace
    foreach(path ${ARGN})
      string(LENGTH "${path}" path_len)
      foreach(i RANGE ${CATKIN_ORDERED_SPACES_END})
        list(GET CATKIN_ORDERED_SPACES ${i} space)
        string(LENGTH "${space}" len)

        # If the path is long enough, compare the prefix
        if(${len} LESS ${path_len})
          string(SUBSTRING "${path}" 0 ${len} path_prefix)
          if("${space}" STREQUAL "${path_prefix}")
            list(APPEND slot_${i} "${path}")
            break()
          endif()
        endif()
      endforeach()
    endforeach()

    # Flatten the slot list
    set(${listname} "")
    foreach(i RANGE ${CATKIN_ORDERED_SPACES_END})
        list(APPEND ${listname} ${slot_${i}})
    endforeach()
  else()
    set(${listname} "")
  endif()
endmacro()
