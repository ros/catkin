#!/usr/bin/env python

import sys
from catkintest import *

def setup():
    print "basics ***************************************************************\n"*8
    rosinstall(srcdir, 'test.rosinstall')

def startbuild():
    if isdir(builddir):
        shutil.rmtree(builddir)
    succeed("mkdir %s" % builddir)

def endbuild():
    pass

def teardown():
    pass

bt = with_setup(startbuild, endbuild)

@bt
def test_tiny():
    out = cmake(CATKIN_BUILD_PROJECTS='nolangs',
                CMAKE_INSTALL_PREFIX=cmake_install_prefix)
    assert exists(builddir + "/nolangs")
    assert not exists(builddir + "/std_msgs")
    assert not exists(builddir + "/genmsg")
    out = succeed("make VERBOSE=1", cwd=builddir)
    assert exists(builddir + "/bin/nolangs_exec")

    out = succeed("make VERBOSE=1 install DESTDIR=%s" % destdir, cwd=builddir)

    assert_exists(diskprefix,
                  "bin/nolangs_exec",
                  "share/cmake/nolangs/nolangs-config.cmake")

    out = succeed("make help", cwd=builddir)
    out = succeed("make VERBOSE=1 nolangs-gendebian", cwd=builddir)
    
    assert exists(builddir + "/nolangs/catkin-test-nolangs_3.4.5~natty.dsc")
    assert exists(builddir + "/nolangs/catkin-test-nolangs_3.4.5~natty.tar.gz")

@bt
def test_00():
    out = cmake(CMAKE_INSTALL_PREFIX=cmake_install_prefix)
    out = succeed("make", cwd=builddir)
    out = succeed("make install DESTDIR=%s" % destdir, cwd=builddir)
    prefix = "%s/%s/%s" % (builddir, destdir, cmake_install_prefix)

    assert_exists(builddir,
                  "lib/liba.so",
                  "lib/libb.so",
                  "lib/libc.so",
                  "lib/libd.so",
                  "nolangs",
                  "bin/nolangs_exec",
                  "bin/quux_srv-exec",
                  "genmsg",
                  "common_msgs")
    assert_exists(builddir+'/gen/py',
                  "a", "b", "c", "d",
                  "a/__init__.py")

    assert_exists(diskprefix,
                  "bin/nolangs_exec",
                  "include/std_msgs/String.h",
                  pyinstall + "/sensor_msgs/msg/_PointCloud2.py",
                  "share/msg/std_msgs/String.msg",
                  "share/cmake/std_msgs/std_msgs-config.cmake",
                  "share/cmake/std_msgs/std_msgs-config-version.cmake",
                  "lib/libcpp_common.so")

@bt
def test_noproject():
    mybuild = pwd + '/build/ck'
    if isdir(mybuild):
        shutil.rmtree(mybuild)
    os.makedirs(mybuild)
    INST = pwd + "/build/INST"
    out = cmake(CATKIN_BUILD_PROJECTS='catkin',
                CMAKE_INSTALL_PREFIX=INST,
                cwd=mybuild)
    out = succeed("make install", cwd=mybuild)
    shutil.rmtree(mybuild)
    os.makedirs(mybuild)

    out = cmake(srcdir=pwd+'/src/noproject',
                cwd=mybuild,
                CMAKE_PREFIX_PATH=INST,
                expect=fail)
    print "failed as expected, out=", out
    assert 'macro to specify the name of your project' in out

#
# This one needs love:  catkin looks in wrong directory for buildable projects
#
#@attr('this')
#def test_as_subdirectory():
#    succeed("rosinstall -n subdir_src/src test.rosinstall")
#    succeed("rm -rf subdir_build")
#    succeed("mkdir subdir_build")
#    cmake(this_srcdir="subdir_src", cwd="subdir_build")


# @with_setup(startbuild, endbuild)
# def test_01():
#     out = cmake()
#
#     out = succeed("make", cwd=builddir)
#     print "OUT=", out
#     assert exists(builddir + "/nolangs/nolangs_exec")



