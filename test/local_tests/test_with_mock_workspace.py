#!/usr/bin/env python

import os
import shutil
from test.utils import AbstractCatkinWorkspaceTest, MOCK_DIR, \
    MAKE_CMD, succeed, assert_exists, fail


class MockTest(AbstractCatkinWorkspaceTest):
    """
    This test case uses workspaces with catkin projects from the
    test/mock_resources folder.
    """

    # uncomment to keep temporary files in /tmp
    # def tearDown(self):
    #     pass

    def test_catkin_only(self):
        self.cmake()
        succeed(MAKE_CMD, cwd=self.builddir)
        succeed(MAKE_CMD + ["install"], cwd=self.builddir)

        assert_exists(self.installdir,
                      "env.sh",
                      "setup.sh",
                      "setup.zsh")

    def test_nolang(self):
        dstdir = os.path.join(self.workspacedir, 'nolangs')
        shutil.copytree(os.path.join(MOCK_DIR, 'src', 'nolangs'), dstdir)

        out = self.cmake(CATKIN_WHITELIST_PACKAGES='nolangs',
                         CATKIN_DPKG_BUILDPACKAGE_FLAGS='-d;-S;-us;-uc')
        self.assertTrue(os.path.exists(self.builddir + "/nolangs"))
        self.assertFalse(os.path.exists(self.builddir + "/std_msgs"))
        self.assertFalse(os.path.exists(self.builddir + "/genmsg"))
        out = succeed(MAKE_CMD, cwd=self.builddir)
        self.assertTrue(os.path.exists(self.builddir +
                                       "/nolangs/bin/nolangs_exec"))
        out = succeed(MAKE_CMD + ["install"], cwd=self.builddir)

        assert_exists(self.installdir,
                      "bin/nolangs_exec",
                      "share/nolangs/cmake/nolangsConfig.cmake")

        # also test make help
        succeed(MAKE_CMD + ["help"], cwd=self.builddir)

    def test_noproject(self):
        # create workspace with just catkin and 'noproject' project
        dstdir = os.path.join(self.workspacedir, 'noproject')
        shutil.copytree(os.path.join(MOCK_DIR, 'src-fail', 'noproject'), dstdir)
        # test with whitelist
        out = self.cmake(CATKIN_WHITELIST_PACKAGES='catkin')
        out = succeed(MAKE_CMD + ["install"], cwd=self.builddir)

        shutil.rmtree(self.builddir)
        # fail if we try to build noproject stack
        os.makedirs(self.builddir)

        out = self.cmake(CMAKE_PREFIX_PATH=self.installdir,
                         expect=fail)
        print("failed as expected, out=", out)

        assert 'catkin_project() CATKIN_CURRENT_PACKAGE is not set.' in out
        # assert 'You must call project() with the same name before.' in out

    # Test was not finished apparently
    # def test_help_bad_changelog(self):
    #     self.cmake(CATKIN_ENABLE_DEBBUILDING='TRUE',
    #           CMAKE_PREFIX_PATH=diskprefix,
    #           srcdir=os.path.join(MOCK_DIR,
    #                               'src-fail', 'badly_specified_changelog'),
    #           CATKIN='YES')
    #     succeed(MAKE_CMD + ['help'], cwd=self.builddir)
