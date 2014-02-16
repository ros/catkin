
include(../debug_message.cmake)
include(../list_insert_in_workspace_order.cmake)

function(test_insert_in_workspace_order_single workspaces input expected_output)
  # Setup fake workspace list
  set(CATKIN_ORDERED_SPACES "${workspaces}")
  list(LENGTH CATKIN_ORDERED_SPACES CATKIN_ORDERED_SPACES_END)
  math(EXPR CATKIN_ORDERED_SPACES_END "${CATKIN_ORDERED_SPACES_END} - 1")

  list_insert_in_workspace_order(output ${input})

  if(NOT "${output}" STREQUAL "${expected_output}")
    message(FATAL_ERROR "Test ${input} -> ${expected_output} failed: got ${output}")
  endif()
endfunction()

test_insert_in_workspace_order_single("foo;bar" "" "")
test_insert_in_workspace_order_single("foo;bar"
  "foo3;bar/1;foo/1;foo/2;bar/2"
  "foo/1;foo/2;bar/1;bar/2;foo3"
)
test_insert_in_workspace_order_single(""
  "foo/1;foo/2"
  "foo/1;foo/2"
)
test_insert_in_workspace_order_single(""
  "foo/1;foo/2"
  "foo/1;foo/2"
)
