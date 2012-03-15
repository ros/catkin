#!/usr/bin/env python

import sys, platform
from catkintest import *

ubuntudist = platform.dist()[2]

def setup():
    print "basics ***************************************************************\n"*8
    rosinstall(srcdir, 'test.rosinstall')

def startbuild():
    if isdir(builddir):
        shutil.rmtree(builddir)
    succeed(["/bin/mkdir", builddir])

def endbuild():
    pass

def teardown():
    pass

bt = with_setup(startbuild, endbuild)

@bt
def test_tiny():
    out = cmake(CATKIN_BUILD_STACKS='nolangs',
                CMAKE_INSTALL_PREFIX=cmake_install_prefix,
                CATKIN_DPKG_BUILDPACKAGE_FLAGS='-d;-S;-us;-uc')
    assert exists(builddir + "/nolangs")
    assert not exists(builddir + "/std_msgs")
    assert not exists(builddir + "/genmsg")
    out = succeed(make, cwd=builddir)
    assert exists(builddir + "/bin/nolangs_exec")

    out = succeed(make + ["install", "DESTDIR=" + destdir], cwd=builddir)

    assert_exists(diskprefix,
                  "bin/nolangs_exec",
                  "share/nolangs/cmake/nolangs-config.cmake")

    out = succeed(make + ["help"], cwd=builddir)

@bt
def test_00():
    out = cmake(CMAKE_INSTALL_PREFIX=cmake_install_prefix)
    out = succeed(make, cwd=builddir)
    out = succeed(make + ["install", "DESTDIR=" + destdir], cwd=builddir)
    prefix = "%s/%s/%s" % (builddir, destdir, cmake_install_prefix)

    assert_exists(builddir,
                  "lib/liba.so",
                  "lib/libb.so",
                  "lib/libc-one.so",
                  "lib/libc-two.so",
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
                  "share/std_msgs/msg/String.msg",
                  "share/std_msgs/cmake/std_msgs-config.cmake",
                  "share/std_msgs/cmake/std_msgs-config-version.cmake",
                  "lib/libcpp_common.so")

    #
    #  make sure python imports work
    #
    succeed([builddir + "/env.sh", pwd + "/import_a.py"])
    succeed([builddir + "/env.sh", pwd + "/import_b.py"])


@bt
def test_noproject():
    mybuild = pwd + '/build/ck'
    if isdir(mybuild):
        shutil.rmtree(mybuild)
    os.makedirs(mybuild)
    INST = pwd + "/build/INST"
    out = cmake(CATKIN_BUILD_STACKS='catkin',
                CMAKE_INSTALL_PREFIX=INST,
                cwd=mybuild)
    out = succeed(["/usr/bin/make", "install"], cwd=mybuild)
    shutil.rmtree(mybuild)
    os.makedirs(mybuild)

    out = cmake(srcdir=pwd+'/src-fail/noproject',
                cwd=mybuild,
                CMAKE_PREFIX_PATH=INST,
                expect=fail)
    print "failed as expected, out=", out
    assert 'macro to specify the name of your project' in out


@bt
def test_strangelanguage():
    mybuild = pwd + '/build/typelib'
    if isdir(mybuild):
        shutil.rmtree(mybuild)
    os.makedirs(mybuild)
    INST = pwd + "/build/TLINST"
    out = cmake(CATKIN_BUILD_STACKS='catkin;genmsg;gentypelibxml;std_msgs',
                CMAKE_INSTALL_PREFIX=INST,
                cwd=mybuild)
    out = succeed(["/usr/bin/make", "install"], cwd=mybuild)
    shutil.rmtree(mybuild)
    os.makedirs(mybuild)

    assert_exists(INST, 'share/typelibxml/std_msgs/Float32.xml')

@bt
def test_strangelanguage_installed():
    mybuild = pwd + '/build/typelib'
    if isdir(mybuild):
        shutil.rmtree(mybuild)
    os.makedirs(mybuild)
    INST = pwd + "/build/TLINST"
    out = cmake(CATKIN_BUILD_STACKS='catkin;genmsg;gentypelibxml',
                CMAKE_INSTALL_PREFIX=INST,
                cwd=mybuild)
    out = succeed(["/usr/bin/make", "install"], cwd=mybuild)
    assert_exists(INST,
                  'etc/langs/gentypelibxml',
                  'share/gentypelibxml/cmake/gentypelibxml.cmake')

    shutil.rmtree(mybuild)
    os.makedirs(mybuild)

    out = cmake(srcdir=pwd+'/src/std_msgs',
                cwd=mybuild,
                CMAKE_PREFIX_PATH=INST,
                CMAKE_INSTALL_PREFIX=INST)

    out = succeed(["/usr/bin/make", "install"], cwd=mybuild)
    assert_exists(INST,
                  'share/typelibxml/std_msgs/Float32.xml')


@bt
@attr('this')
def test_common_msgs_against_installed():
    mybuild = pwd + '/build/cmsgs'
    if isdir(mybuild):
        shutil.rmtree(mybuild)
    os.makedirs(mybuild)
    INST = pwd + "/INST/cmsgs-inst"
    if isdir(INST):
        shutil.rmtree(INST)
    out = cmake(CATKIN_BUILD_STACKS='catkin;genmsg;genpy;gencpp;gentypelibxml;genpybindings;std_msgs;roscpp_core',
                CMAKE_INSTALL_PREFIX=INST,
                cwd=mybuild)
    out = succeed(["/usr/bin/make", "install"], cwd=mybuild)

    assert_exists(INST,
                  'etc/langs/genpy')

    shutil.rmtree(mybuild)
    os.makedirs(mybuild)

    out = cmake(srcdir=pwd+'/src/common_msgs',
                cwd=mybuild,
                CATKIN='YES',
                CMAKE_PREFIX_PATH=INST,
                CMAKE_INSTALL_PREFIX=INST)

    out = succeed(['/usr/bin/make', 'VERBOSE=1'], cwd=mybuild)
    assert_exists(mybuild,
                  'gen/py/nav_msgs/msg/_GridCells.py',
                  'gen/cpp/nav_msgs/GridCells.h')

    out = succeed(['/usr/bin/make', 'install', 'VERBOSE=1'], cwd=mybuild)

    assert_exists(INST,
                  'include/geometry_msgs/PointStamped.h',
                  pyinstall + '/geometry_msgs/msg/_PointStamped.py',
                  'share/geometry_msgs/msg/PointStamped.msg',
                  'share/typelibxml/geometry_msgs/PointStamped.xml',

        )

