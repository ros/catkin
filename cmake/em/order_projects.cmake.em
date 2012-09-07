# generated from catkin/cmake/em/order_projects.cmake.em
@{
import os
from catkin.topological_order import get_message_generators, topological_order
ordered_stacks = topological_order(os.path.normpath(source_root_dir), whitelisted_stacks, blacklisted_stacks)
}@

set(CATKIN_ORDERED_PROJECTS "")
set(CATKIN_ORDERED_PROJECT_PATHS "")
@[for name, project_data in ordered_stacks]@
@[if not name]@
message(FATAL_ERROR "Circular dependency in subset of projects:\n@project_data")
@[end if]@
list(APPEND CATKIN_ORDERED_PROJECTS @(name))
list(APPEND CATKIN_ORDERED_PROJECT_PATHS @(project_data.path.replace(os.sep, '/')))
@[end for]@

@{
message_generators = get_message_generators(ordered_stacks)
}@
set(CATKIN_MESSAGE_GENERATORS @(' '.join(message_generators)))
