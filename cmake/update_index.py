#!/usr/bin/env python

from __future__ import print_function
import os, sys, yaml

pkgfile = sys.argv[1]
if os.path.exists(pkgfile):
    y = dict([l.split() for l in open(pkgfile).readlines()])
else:
    y = {}

y[sys.argv[2]] = sys.argv[3]

ofile = open(pkgfile, 'w')
for k, v in y.items():
    print("%s %s" % (k, v), file=ofile)




