#!/usr/bin/env python

import os, sys, yaml

pkgfile = sys.argv[1]
if os.path.exists(pkgfile):
    y = yaml.load(open(pkgfile))
else:
    y = {}

y[sys.argv[2]] = sys.argv[3]

ofile = open(pkgfile, 'w')
yaml.dump(y, ofile)
ofile.close()


