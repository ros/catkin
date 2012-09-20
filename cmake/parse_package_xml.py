#!/usr/bin/env python

from __future__ import print_function
import os
import sys

try:
    from catkin.package import parse_package
except ImportError:
    # find the import relatively to make it work before installing catkin
    sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'python'))
    from catkin.package import parse_package

package = parse_package(sys.argv[1])

values = {}
values['version'] = package.version

maintainers = []
for m in package.maintainers:
    maintainer = m['name']
    if m['email'] is not None:
        maintainer += ' <%s>' % m['email']
    maintainers.append(maintainer)
values['maintainer'] = ', '.join(maintainers)

# need to turn the depends into semicolon separated list
values['depends'] = ';'.join([d['name'] for d in (package.build_depends + package.buildtool_depends)])

with open(sys.argv[2], 'w') as ofile:
    print(r'set(_CATKIN_CURRENT_PACKAGE "%s")' % package.name, file=ofile)
    for k, v in values.items():
        print('set(%s_%s "%s")' % (package.name, k.upper(), v), file=ofile)
