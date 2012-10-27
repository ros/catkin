import os
import unittest
import tempfile
import shutil

import imp
imp.load_source('setup_util',
                os.path.join(os.path.dirname(__file__),
                             '..', '..', 'cmake', 'templates', 'setup_util.py'))

import setup_util
from setup_util import get_reversed_workspaces, prefix_env, remove_from_env, CATKIN_WORKSPACE_MARKER_FILE


class SetupUtilTest(unittest.TestCase):

    def test_get_reversed_workspaces(self):
        try:
            mock_env = {}
            rootdir = tempfile.mkdtemp()
            setup_util.os.environ = mock_env
            self.assertEqual('', get_reversed_workspaces())
            self.assertEqual('', get_reversed_workspaces('foo'))
            foows = os.path.join(rootdir, 'foo')
            os.makedirs(foows)
            with open(os.path.join(foows, CATKIN_WORKSPACE_MARKER_FILE), 'w') as fhand:
                fhand.write('')
            barws = os.path.join(rootdir, 'bar')
            os.makedirs(barws)
            with open(os.path.join(barws, CATKIN_WORKSPACE_MARKER_FILE), 'w') as fhand:
                fhand.write('')
            nows = os.path.join(rootdir, 'nows')
            os.makedirs(nows)
            mock_env = {'CMAKE_PREFIX_PATH': os.pathsep.join([nows, foows, barws, 'invalid'])}
            setup_util.os.environ = mock_env
            self.assertEqual('\n'.join([barws, foows]), get_reversed_workspaces())
            self.assertEqual(barws, get_reversed_workspaces(foows))
            self.assertEqual(foows, get_reversed_workspaces(barws))
        finally:
            shutil.rmtree(rootdir)
            setup_util.os.environ = os.environ

    def test_prefix_env(self):
        try:
            mock_env = {}
            setup_util.os.environ = mock_env
            self.assertEqual('',
                             prefix_env('varname', ''))
            self.assertEqual(os.pathsep.join(['foo', 'bar']),
                             prefix_env('varname', 'foo:bar'))
            mock_env = {'varname': os.pathsep.join(['baz', 'bar', 'bam'])}
            setup_util.os.environ = mock_env
            self.assertEqual('',
                             prefix_env('varname', ''))
            self.assertEqual('foo' + os.pathsep,
                             prefix_env('varname', 'foo:bar'))
            self.assertEqual(os.pathsep.join(['foo', 'lim']) + os.pathsep,
                             prefix_env('varname', 'foo:lim:foo:lim'))
        finally:
            setup_util.os.environ = os.environ

    def test_remove_from_env(self):
        try:
            mock_env = {}
            rootdir = tempfile.mkdtemp()
            setup_util.os.environ = mock_env
            foows = os.path.join(rootdir, 'foo')
            foolib = os.path.join(foows, 'lib')
            os.makedirs(foows)
            with open(os.path.join(foows, '.CATKIN_WORKSPACE'), 'w') as fhand:
                fhand.write('')
            barws = os.path.join(rootdir, 'bar')
            barlib = os.path.join(barws, 'lib')
            os.makedirs(barws)
            with open(os.path.join(barws, '.CATKIN_WORKSPACE'), 'w') as fhand:
                fhand.write('')
            nows = os.path.join(rootdir, 'nows')
            nowslib = os.path.join(nows, 'lib')
            os.makedirs(nows)
            setup_util.os.environ = mock_env
            mock_env = {'varname': os.pathsep.join([foolib, barlib])}
            setup_util.os.environ = mock_env
            self.assertEqual(os.pathsep.join([foolib, barlib]), remove_from_env('varname', ''))
            self.assertEqual(os.pathsep.join([foolib, barlib]), remove_from_env('varname', 'child1'))

            mock_env = {'varname': os.pathsep.join([foolib, nowslib, barlib, foolib]), 'CMAKE_PREFIX_PATH': os.pathsep.join([foows, barws])}
            setup_util.os.environ = mock_env
            # check nows/lib remains, and second mention of foolib
            self.assertEqual(os.pathsep.join([nowslib, foolib]), remove_from_env('varname', 'lib'))

        finally:
            setup_util.os.environ = os.environ
