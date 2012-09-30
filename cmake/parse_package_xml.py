#!/usr/bin/env python

from __future__ import print_function
import sys

from catkin_pkg.package import parse_package

package = parse_package(sys.argv[1])

values = {}
values['VERSION'] = '"%s"' % package.version

values['MAINTAINER'] = '"%s"' % (', '.join([str(m) for m in package.maintainers]))

values['BUILD_DEPENDS'] = ' '.join(['"%s"' % str(d) for d in package.build_depends])
values['RUN_DEPENDS'] = ' '.join(['"%s"' % str(d) for d in package.run_depends])

with open(sys.argv[2], 'w') as ofile:
    print(r'set(_CATKIN_CURRENT_PACKAGE "%s")' % package.name, file=ofile)
    for k, v in values.items():
        print('set(%s_%s %s)' % (package.name, k, v), file=ofile)
