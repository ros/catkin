#!/usr/bin/env python

from __future__ import print_function
import pprint, sys, os

# print("%s" % sys.argv)
PREFIX = sys.argv[1]
outfile = sys.argv[3]
out = open(outfile, "w")
#print("Interrogating setup.py for package %s into %s " % (PREFIX, outfile),
#      file=sys.stderr)

def mysetup(*args, **kwargs):
    global out
    if 'version' not in kwargs:
        print("""
    *** Unable to find 'version' in setup.py of %s
""" % PREFIX)
        raise RuntimeError("version not found in setup.py")

    print(r'set(%s_VERSION "%s")' % (PREFIX, kwargs['version']), file=out)
    print(r'set(%s_SCRIPTS "%s")' % (PREFIX,
                                     ';'.join(kwargs.get('scripts', []))), file=out)
    pkg_dir = kwargs.get('package_dir', {'':''})
    allprefix=''
    if '' in pkg_dir:
        allprefix=pkg_dir['']

    pkgs = kwargs.get('packages', [])
    resolved_pkgs = []
    for pkg in pkgs:
        if allprefix:
            resolved_pkgs += [os.path.join(allprefix, pkg)]
        else:
            resolved_pkgs += []
    print(r'set(%s_PACKAGES "%s")' % (PREFIX, ';'.join(pkgs)), file=out)

    if 'package_dir' not in kwargs:
        print(r'package_dir not in setup.py', file=sys.stderr)
        sys.exit(1)
    if '' not in kwargs['package_dir'].keys():
        print(r'Error: empty string not in package_dir: %s\n(not yet supported)' % kwargs['package_dir'],
              file=sys.stderr)
        sys.exit(1)

    print(r'set(%s_PACKAGE_DIR "%s")' % (PREFIX, kwargs['package_dir']['']), file=out)

class Dummy: pass

d = Dummy()
setattr(d, 'setup', mysetup)

sys.modules['setuptools'] = d
sys.modules['distutils.core'] = d

# print("execcing %s" % sys.argv[2])

# be sure you're in the directory containing
# setup.py so the sys.path manipulation works,
# so the import of __version__ works
os.chdir(os.path.dirname(os.path.abspath(sys.argv[2])))

execfile(sys.argv[2])






