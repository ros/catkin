#!/usr/bin/env python

from __future__ import print_function
import rospkg.stack
import sys

stack = rospkg.stack.parse_stack_file(sys.argv[1])

pkg = stack.name

values = {}
for k in ['version', 'maintainer']:
    if hasattr(stack, k):
        values[k] = getattr(stack, k)
# need to turn the depends into semicolon separated list
values['depends'] = ';'.join(stack.build_depends)

with open(sys.argv[2], 'w') as ofile:
    for k, v in values.items():
        print('set(%s_%s "%s" CACHE INTERNAL "" FORCE)' % (pkg, k.upper(), v), file=ofile)
    print(r'set(CATKIN_CURRENT_STACK %s CACHE INTERNAL "" FORCE)'  % pkg, file=ofile)
