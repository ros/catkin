#
# Append elements to a list and remove existing duplicates from the list.
#
# .. note:: Using CMake's ``list(APPEND ..)`` and
#   ``list(REMOVE_DUPLICATES ..)`` is not sufficient since its
#   implementation uses a set internally which makes the operation
#   unstable.
#
macro(list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    list(REMOVE_ITEM ${listname} ${ARGN})
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()
