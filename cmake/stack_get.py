#!/usr/bin/env python

from __future__ import print_function
import yaml, sys

fields = ['Version', 'Maintainer']

y = yaml.load(open(sys.argv[1]))

pkg = y['Catkin-ProjectName']

ofile = open(sys.argv[2], 'w')

for k, v in y.items():
    if k in fields:
        print(r'set(%s_%s "%s" CACHE INTERNAL "" FORCE)' % (pkg, k.upper(), v), file=ofile)

print(r'set(CATKIN_CURRENT_STACK %s CACHE INTERNAL "" FORCE)' %pkg, file=ofile)

