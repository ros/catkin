import os
import unittest
import tempfile
import shutil

import imp
imp.load_source('setup_util',
                os.path.join(os.path.dirname(__file__),
                             '..', '..', 'cmake', 'templates', '_setup_util.py'))

import setup_util
from setup_util import get_reversed_workspaces, prefix_env, remove_from_env, CATKIN_MARKER_FILE


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
            with open(os.path.join(foows, CATKIN_MARKER_FILE), 'w') as fhand:
                fhand.write('')
            barws = os.path.join(rootdir, 'bar')
            os.makedirs(barws)
            with open(os.path.join(barws, CATKIN_MARKER_FILE), 'w') as fhand:
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
        altsep = os.path.altsep
        try:
            mock_env = {}
            rootdir = tempfile.mkdtemp()
            setup_util.os.environ = mock_env
            # foows
            foows = os.path.join(rootdir, 'foo')
            foolib = os.path.join(foows, 'lib') + '/'
            os.makedirs(foows)
            with open(os.path.join(foows, '.catkin'), 'w') as fhand:
                fhand.write('')
            # barws
            barws = os.path.join(rootdir, 'bar')
            barlib = os.path.join(barws, 'lib')
            os.makedirs(barws)
            with open(os.path.join(barws, '.catkin'), 'w') as fhand:
                fhand.write('')
            # mock_env with one ws in CPP
            varname = 'varname'
            wsvarname = 'workspaces'
            mock_env = {varname: os.pathsep.join([foolib, barlib]),
                        'CMAKE_PREFIX_PATH': barws}
            setup_util.os.environ = mock_env
            # since workspace foo is not in CMAKE_PREFIX_PATH, it remains in varname
            self.assertEqual(foolib, remove_from_env(varname, '/lib'))

            # mock_env with both ws in CPP
            mock_env = {varname: os.pathsep.join([foolib, barlib]),
                        wsvarname: os.pathsep.join([foows, barws]),
                        'CMAKE_PREFIX_PATH': os.pathsep.join([foows, barws])}
            setup_util.os.environ = mock_env

            self.assertEqual(os.pathsep.join([foolib, barlib]), remove_from_env(varname, ''))
            self.assertEqual(os.pathsep.join([foolib, barlib]), remove_from_env(varname, 'nolib'))
            self.assertEqual(os.pathsep.join([foolib, barlib]), remove_from_env(varname, '/nolib'))
            self.assertEqual('', remove_from_env(varname, 'lib'))
            self.assertEqual('', remove_from_env(varname, '/lib'))
            self.assertEqual(os.pathsep.join([foolib, barlib]), remove_from_env(varname, ''))
            self.assertEqual('', remove_from_env(wsvarname, ''))

            # nows: not a workspace
            nows = os.path.join(rootdir, 'nows')
            nowslib = os.path.join(nows, 'lib')
            nowslib = os.path.join(nows, 'include')
            os.makedirs(nows)

            mock_env = {'varname': os.pathsep.join([foolib, nowslib, barlib, foolib]),
                        'CMAKE_PREFIX_PATH': os.pathsep.join([foows, barws])}
            setup_util.os.environ = mock_env
            # checks nows/lib remains, and second mention of foolib
            self.assertEqual(os.pathsep.join([nowslib, foolib]), remove_from_env('varname', '/lib'))
            self.assertEqual(os.pathsep.join([nowslib, foolib]), remove_from_env('varname', 'lib'))

            # windows pathsep
            os.path.altsep = '\\'
            self.assertEqual(os.pathsep.join([nowslib, foolib]), remove_from_env('varname', '\\lib'))
        finally:
            setup_util.os.environ = os.environ
            os.path.altsep = altsep
            shutil.rmtree(rootdir)
