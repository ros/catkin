from sys import version_info as v
import subprocess
# from pexpect import *
from nose.tools import *
from nose.plugins.attrib import attr
from os.path import *
import os
import shutil

pwd = os.path.dirname(os.path.dirname(__file__))
os.chdir(pwd)
print "pwd=", pwd
srcdir = os.path.join(pwd, 'src')
builddir = os.path.join(pwd, 'build')
destdir='DESTDIR'
cmake_install_prefix='/CMAKE_INSTALL_PREFIX'
diskprefix="%s/%s/%s" % (builddir, destdir, cmake_install_prefix)
pyinstall='lib/python%u.%u/dist-packages' % (v[0], v[1])
make=['make', 'VERBOSE=1']

def run(args, **kwargs):
    print "run:", args
    p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         cwd=kwargs.get('cwd', None))
    print "P==", p.__dict__
    (stdout, stderr) = p.communicate()
    return (p.returncode, stdout, stderr)

def succeed(cmd, **kwargs):
    print ">>>", cmd, kwargs
    (r, out, err) = run(cmd, **kwargs)
    print "<<<", out
    if r != 0:
        print "cmd failed: " + str(cmd)
        print "result=", r
        assert r == 0
    return out

def rosinstall(pth, specfile):
    assert exists(pth)
    assert exists(specfile)
    succeed(["rosinstall", "-n", pth, specfile], cwd=pwd)
    assert( exists(pth + "/catkin/cmake/toplevel.cmake"))
    succeed(["/bin/rm", "-f", "CMakeLists.txt"], cwd=pth)
    succeed(["/bin/ln", "-s", "catkin/cmake/toplevel.cmake", "CMakeLists.txt"], cwd=pth)

def fail(cmd, **kwargs):
    print ">>>", cmd, kwargs
    (r, out, err) = run(cmd, withexitstatus=True, **kwargs)
    print "<<<", out
    if r == 0:
        print "cmd failed: %s\n  result=%u\n  output=\n%s" % (cmd, r, out)
        assert r != 0
    return out

def has_cmakecache():
    assert isfile(builddir + "/CMakeCache.txt")

def cmake(**kwargs):
    args = []
    this_builddir = builddir
    this_srcdir = srcdir
    expect = succeed
    print "v~_", this_builddir, this_srcdir
    if 'CATKIN_DPKG_BUILDPACKAGE_FLAGS' not in kwargs:
        kwargs['CATKIN_DPKG_BUILDPACKAGE_FLAGS'] = '-d;-S;-us;-uc'

    for k, v in kwargs.items():
        print "~v^v~", k, v
        if k == 'cwd':
            this_builddir = v
        elif k == 'srcdir':
            this_srcdir = v
        elif k == 'expect':
            print "USING EXPECT=", v
            expect = v
        else:
            args += ["-D%s=%s" % (k,v)]

    if not isdir(this_builddir):
        os.makedirs(this_builddir)
    cmd = ["cmake", this_srcdir] + args
    o = expect(cmd, cwd=this_builddir)
    if (expect == succeed):
        assert isfile(this_builddir + "/CMakeCache.txt")
        assert isfile(this_builddir + "/Makefile")
    return o

def assert_exists(prefix, *args):
    for arg in args:
        p = os.path.join(prefix, arg)
        print "Checking for", p
        assert exists(p), "%s doesn't exist" % p


