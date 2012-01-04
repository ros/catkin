#!/usr/bin/env python

from __future__ import print_function
import pprint, sys, os

# print("%s" % sys.argv)
STACKNAME = sys.argv[1]
outfile = sys.argv[3]
out = open(outfile, "w")
#print("Interrogating setup.py for package %s into %s " % (STACKNAME, outfile),
#      file=sys.stderr)

def mysetup(*args, **kwargs):
    global out
    if 'version' not in kwargs:
        print("""
    *** Unable to find 'version' in setup.py of %s
""" % STACKNAME)
        raise RuntimeError("version not found in setup.py")

    print(r'set(%s_VERSION "%s")' % (STACKNAME, kwargs['version']), file=out)
    print(r'set(%s_SCRIPTS "%s")' % (STACKNAME,
                                     ';'.join(kwargs.get('scripts', []))), file=out)

    if 'package_dir' not in kwargs:
        raise RuntimeError(r'package_dir not in setup.py', file=sys.stderr)

    package_dir = kwargs['package_dir']
    allprefix=package_dir.get('', None)

    pkgs = kwargs.get('packages', [])
    resolved_pkgs = []
    for pkg in pkgs:
        if allprefix:
            resolved_pkgs += [os.path.join(allprefix, pkg)]
        else:
            resolved_pkgs += [package_dir[pkg]]
    print(r'set(%s_PACKAGES "%s")' % (STACKNAME, ';'.join(resolved_pkgs)), file=out)

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






