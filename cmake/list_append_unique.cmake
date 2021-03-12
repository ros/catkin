#
# Append elements to a list if they are not already in the list.
#
macro(list_append_unique listname)
  if(NOT "${ARGN}" STREQUAL "")
    list(APPEND ${listname} ${ARGN})
    list(REMOVE_DUPLICATES ${listname})
  endif()
endmacro()
