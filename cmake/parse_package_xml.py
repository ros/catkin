#!/usr/bin/env python

from __future__ import print_function
import sys

from catkin_pkg.package import parse_package

package = parse_package(sys.argv[1])

values = {}
values['version'] = package.version

values['maintainer'] = ', '.join([str(m) for m in package.maintainers])

# need to turn the depends into semicolon separated list
values['depends'] = ';'.join([d.name for d in (package.build_depends + package.buildtool_depends)])

with open(sys.argv[2], 'w') as ofile:
    print(r'set(_CATKIN_CURRENT_PACKAGE "%s")' % package.name, file=ofile)
    is_meta_package = 'meta_package' in [e.tagname for e in package.exports]
    print(r'set(_CATKIN_CURRENT_PACKAGE_IS_META "%s")' % str(is_meta_package), file=ofile)
    for k, v in values.items():
        print('set(%s_%s "%s")' % (package.name, k.upper(), v), file=ofile)
