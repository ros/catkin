# generated from catkin/cmake/em/sort_stacks_topologically.cmake.em
@{
from catkin.topological_order import get_message_generators, topological_order

ordered_stacks = topological_order(source_root_dir, whitelisted_stacks, blacklisted_stacks)
message_generators = get_message_generators(ordered_stacks)
}@

set(CATKIN_MESSAGE_GENERATORS @(' '.join(message_generators)))

set(CATKIN_TOPOLOGICALLY_SORTED_STACKS "")
@[for name, stack_data in ordered_stacks]@
@[if not name]@
message(FATAL_ERROR "Circular dependency in subset of stacks:\n@stack_data")
@[end if]@
list(APPEND CATKIN_TOPOLOGICALLY_SORTED_STACKS @(stack_data.path))
@[end for]@
