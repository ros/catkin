#!/usr/bin/env python

import os
import shutil
import tempfile
from test.utils import MOCK_DIR, PYTHON_INSTALL_PATH, \
    MAKE_CMD, succeed, assert_exists, create_catkin_workspace
from . import network_test_utils
from network_test_utils import AbstractFuerteTest


class SimpleFuerteTest(AbstractFuerteTest):

    def __init__(self, testCaseName):
        super(SimpleFuerteTest, self).__init__(testCaseName, 'fuerte_test')

    def setUp(self):
        super(SimpleFuerteTest, self).setUp()
        # a bit dirty here, tester needs to manually delete
        # test/tmp/fuerte_test/build folder. But else, tests take
        # too long
        if not os.path.exists(os.path.join(self.builddir, "Makefile")):
            self.cmake()
        # if make fails due to DistributionNotFound,
        # tester needs to manually delete test/tmp/fuerte_test/src
        succeed(MAKE_CMD, cwd=self.builddir)

        succeed(MAKE_CMD + ["install"], cwd=self.builddir)

    def test_make(self):
        # uses core ros stacks installed in setUp
        # just checks the result for success
        self.assertTrue(os.path.exists(self.workspacedir))

        assert_exists(self.builddir,
                      'ros')
        assert_exists(self.buildspace,
                      'bin',
                      'etc',
                      'lib',
                      # only in groovy
                      # 'include',
                      # 'share'
                      )

        assert_exists(self.installdir,
                      'bin',
                      'etc',
                      'include',
                      'lib',
                      'share')

    def test_common_msgs_against_installed(self):
        # uses core ros stacks installed in setUp
        # tries to build common_msgs in separate workspace
        try:
            # create another workspace than self.rootdir
            other_root_dir = tempfile.mkdtemp()
            other_src_dir = os.path.join(other_root_dir, 'src')
            create_catkin_workspace(other_src_dir)
            other_build_dir = os.path.join(other_root_dir, 'build')
            other_install_dir = os.path.join(other_root_dir, 'install')
            shutil.copytree(os.path.join(self.workspacedir, 'common_msgs'),
                            os.path.join(other_src_dir, 'common_msgs'))
            cenv=os.environ.copy()
            
            cenv['PYTHONPATH']="%s:%s" % (os.path.join(self.installdir, 'lib', 'python2.7', 'dist-packages'), cenv['PYTHONPATH'])

            prefix_path_ext=[]
            for package in ['genmsg', 'genpy', 'gencpp']:
                prefix_path_ext.append(os.path.join(self.installdir, 'share', package, 'cmake'))
            out = self.cmake(cwd=other_build_dir,
                             env=cenv,
                             # installdir=other_install_dir,
                             prefix_path=':'.join(prefix_path_ext),
                             srcdir=other_src_dir)
            out = succeed(MAKE_CMD, cwd=other_build_dir, env=cenv)
            assert_exists(other_build_dir,
                          'gen/py/nav_msgs/msg/_GridCells.py',
                          'gen/cpp/nav_msgs/GridCells.h')

            out = succeed(MAKE_CMD + ['install'], cwd=other_build_dir, env=cenv)

            assert_exists(self.installdir,
                          'include/geometry_msgs/PointStamped.h',
                          PYTHON_INSTALL_PATH + '/geometry_msgs/msg/_PointStamped.py',
                          'share/geometry_msgs/msg/PointStamped.msg',
                          # 'share/typelibxml/geometry_msgs/PointStamped.xml',
                          )
        finally:
            # pass
            shutil.rmtree(other_root_dir)

    def test_mock_against_installed(self):
        # uses core ros stacks installed in setUp
        # tries to build mock project in separate workspace
        try:
            # create another workspace than self.rootdir
            other_root_dir = tempfile.mkdtemp()
            other_src_dir = os.path.join(other_root_dir, 'src')
            create_catkin_workspace(other_src_dir)
            other_build_dir = os.path.join(other_root_dir, 'build')
            
            other_install_dir = os.path.join(other_root_dir, 'install')
            shutil.copytree(os.path.join(MOCK_DIR, 'src', 'catkin_test'),
                            os.path.join(other_src_dir, 'catkin_test'))
            cenv=os.environ.copy()
            
            cenv['PYTHONPATH']="%s:%s" % (os.path.join(self.installdir, 'lib', 'python2.7', 'dist-packages'), cenv['PYTHONPATH'])
            prefix_path_ext=[]
            for package in ['genmsg', 'genpy', 'gencpp']:
                prefix_path_ext.append(os.path.join(self.installdir, 'share', package, 'cmake'))

            self.cmake(cwd=other_build_dir,
                       env=cenv,
                       srcdir=other_src_dir,
                       installdir=other_install_dir)
            succeed(MAKE_CMD, cwd=other_build_dir, env=cenv)

            assert_exists(other_build_dir,
                          "lib/liba.so",
                          "lib/libb.so",
                          "lib/libc-one.so",
                          "lib/libc-two.so",
                          "lib/libd.so")
            assert_exists(other_build_dir,
                          # "bin/nolangs_exec",
                          "catkin_test/quux_user/bin/quux_srv-exec")
            assert_exists(other_build_dir,
                          "gen/py/a",
                          "gen/py/a/__init__.py")

            # "DESTDIR="
            succeed(MAKE_CMD + ["install"],
                    cwd=other_build_dir)

            assert_exists(other_install_dir,
                          "lib/pkgconfig/a.pc",
                          "lib/pkgconfig/b.pc",
                          "lib/pkgconfig/c.pc",
                          "lib/pkgconfig/d.pc",
                          "lib/pkgconfig/quux_msgs.pc",
                          PYTHON_INSTALL_PATH + "/a/__init__.py",
                          PYTHON_INSTALL_PATH + "/quux_msgs/__init__.py")

            #
            #  make sure python imports work
            #

            # succeed([other_build_dir + "/env.sh", "python -c 'import a'"])
            # succeed([other_build_dir + "/env.sh", "python -c 'import b'"])
        finally:
            pass
            # shutil.rmtree(other_root_dir)
