#!/usr/bin/env python

import sys
from catkintest import *

def setup():
    print "snake ***************************************************************\n"*8
    succeed("rosinstall -n %s test.rosinstall" % srcdir)
    assert exists(srcdir + "/catkin/toplevel.cmake")
    succeed("rm -f CMakeLists.txt", cwd=srcdir)
    succeed("ln -s catkin/toplevel.cmake CMakeLists.txt", cwd=srcdir)
    if isdir(builddir):
        shutil.rmtree(builddir)
    succeed("mkdir %s" % builddir)
    out = cmake()
    out = succeed("make", cwd=builddir)

def startbuild():
    pass

def endbuild():
    pass

def teardown():
    pass

bt = with_setup(startbuild, endbuild)

@bt
def test_00():
    succeed(builddir + "/env.sh " + pwd + "/import_a.py")

@bt
def test_01():
    succeed(builddir + "/env.sh " + pwd + "/import_b.py")






# @with_setup(startbuild, endbuild)
# def test_01():
#     out = cmake()
#
#     out = succeed("make", cwd=builddir)
#     print "OUT=", out
#     assert exists(builddir + "/catkin_test_nolangs/catkin_test_nolangs_exec")



