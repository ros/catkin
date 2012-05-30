#!/usr/bin/env python

from __future__ import print_function
import rospkg.stack
import sys

stack = rospkg.stack.parse_stack_file(sys.argv[1])

pkg = stack.name

values = {}
values['version'] = stack.version

maintainers = []
for m in stack.maintainers:
    maintainer = m.name
    if m.email:
        maintainer += ' <%s>' % m.email
    maintainers.append(maintainer)
values['maintainer'] = ', '.join(maintainers)

# need to turn the depends into semicolon separated list
values['depends'] = ';'.join([d.name for d in stack.build_depends])

with open(sys.argv[2], 'w') as ofile:
    for k, v in values.items():
        print('set(%s_%s "%s" CACHE INTERNAL "" FORCE)' % (pkg, k.upper(), v), file=ofile)
    print(r'set(CATKIN_CURRENT_STACK %s CACHE INTERNAL "" FORCE)'  % pkg, file=ofile)
