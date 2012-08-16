# generated from catkin/cmake/em/sort_stacks_topologically.cmake.em
@{
from __future__ import print_function
import glob
import os
import sys
import rospkg.stack

class StackData:

    def __init__(self, stack_xml_filename):
        stack = rospkg.stack.parse_stack_file(stack_xml_filename)
        self.name = stack.name
        self.path = os.path.dirname(stack_xml_filename).replace('\\', '/')
        self.build_depends = set([d.name for d in stack.build_depends])
        self.message_generator = stack.message_generator

    def __repr__(self):
        return "name=%s path=%s build_depends=%s message_generator=%s\n" % (self.name, self.path, self.build_depends, self.message_generator)

def remove_dependency(stacks, name):
    for build_depends in [stack_data.build_depends for stack_data in stacks.values()]:
        build_depends.difference_update([name])

def sort_stacks(stacks):
    """
    Generating first returning stacks which have message generators and then the rest based on their build_depends.
    """

    def next_stack():
        # find all stack without build dependencies
        message_generators = []
        non_message_generators = []
        for name, stack_data in stacks.items():
            if not stack_data.build_depends:
                if stack_data.message_generator:
                    message_generators.append(name)
                else:
                    non_message_generators.append(name)
        # first choose message generators
        if message_generators:
            stack_names = message_generators
        elif non_message_generators:
            stack_names = non_message_generators
        else:
            return (None, None)
        # alphabetic order only for convenience
        stack_names.sort()
        return (stack_names[0], stacks[stack_names[0]])

    ordered_stacks = []
    while len(stacks) > 0:
        (name, stack_data) = next_stack()
        # in case of a circular dependency pass the list of remaining stacks
        if name is None:
            ordered_stacks.append([None, ', '.join(stacks.keys())])
            break
        ordered_stacks.append([name, stack_data])
        # remove stack from further processing
        del stacks[name]
        remove_dependency(stacks, name)
    return ordered_stacks


stacks = {}
stack_xmls = glob.glob(os.path.join(source_root_dir, '*', 'stack.xml'))
#print('stack_xmls = %s' % stack_xmls, file=sys.stderr)

# fetch all stack data
for stack_xml_filename in stack_xmls:
    if os.path.basename(os.path.dirname(stack_xml_filename)).startswith('.'):
        continue
    stack_data = StackData(stack_xml_filename)
    #print('stack_data: %s' % stack_data, file=sys.stderr)
    # skip non-whitelisted stacks
    if len(whitelisted_stacks) > 0 and stack_data.name not in whitelisted_stacks:
        continue
    # skip blacklisted stacks
    if stack_data.name in blacklisted_stacks:
        continue
    #print('stack_data: %s' % stack_data, file=sys.stderr)
    stacks[stack_data.name] = stack_data

# remove non-stack dependencies from the list
if stacks:
    all_build_depends = reduce(set.union, [p.build_depends for p in stacks.values()])
    non_stack_depends = all_build_depends - set(stacks.keys())
    #print('non_stack_depends = %s' % non_stack_depends, file=sys.stderr)
    for stack_data in stacks.values():
        stack_data.build_depends = set(stack_data.build_depends) - set(non_stack_depends)

# remove catkin from list of stack and build_depends
if 'catkin' in stacks:
    del stacks['catkin']
    remove_dependency(stacks, 'catkin')

message_generators = [name for name, stack_data in stacks.items() if stack_data.message_generator]
message_generators.sort()

ordered_stacks = sort_stacks(stacks)
}@

set(CATKIN_MESSAGE_GENERATORS @(' '.join(message_generators)))

set(CATKIN_TOPOLOGICALLY_SORTED_STACKS "")
@[for name, stack_data in ordered_stacks]@
@[if not name]@
message(FATAL_ERROR "Circular dependency in subset of stacks:\n@stack_data")
@[end if]@
list(APPEND CATKIN_TOPOLOGICALLY_SORTED_STACKS @(stack_data.path))
@[end for]@
