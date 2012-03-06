#!/usr/bin/env python

import sys
from catkintest import *

def setup():
    print "catkin ***************************************************************\n"*8
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
    diskprefix = builddir + cmake_install_prefix
    cmake(CMAKE_INSTALL_PREFIX = diskprefix,
          srcdir=srcdir + '/catkin')
    succeed(["make", "VERBOSE=1"], cwd=builddir)
    succeed(["make", "VERBOSE=1", "install"], cwd=builddir)

    assert_exists(diskprefix,
                  "env.sh",
                  "setup.sh",
                  "setup.zsh")

    succeed(["/bin/rm", "CMakeCache.txt"], cwd=builddir)
    cmake(CATKIN_ENABLE_DEBBUILDING='TRUE',
          CMAKE_PREFIX_PATH=diskprefix,
          srcdir=pwd+'/src-fail/badly_specified_changelog',
          CATKIN='YES')
    succeed(['make', 'VERBOSE=1', 'help'], cwd=builddir)

    succeed(["/bin/rm", "CMakeCache.txt"], cwd=builddir)
    cmake(CATKIN_ENABLE_DEBBUILDING="TRUE",
          CMAKE_PREFIX_PATH=diskprefix,
          srcdir=pwd+'/src/nolangs')
    succeed(['make', 'VERBOSE=1', 'help'], cwd=builddir)

