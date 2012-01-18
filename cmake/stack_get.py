#!/usr/bin/env python

from __future__ import print_function
import yaml, sys

fields = ['Version', 'Maintainer', 'Depends']

y = yaml.load(open(sys.argv[1]))

pkg = y['Catkin-ProjectName']

#need to turn the depends into semicolon seperated list
y['Depends'] = y['Depends'].replace(',',';')

ofile = open(sys.argv[2], 'w')

for k, v in y.items():
    if k in fields:
        print('set(%s_%s "%s" CACHE INTERNAL "" FORCE)' % (pkg, k.upper(), v), file=ofile)

print(r'set(CATKIN_CURRENT_STACK %s CACHE INTERNAL "" FORCE)' %pkg, file=ofile)

