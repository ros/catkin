import os
import shutil
import tempfile
import unittest

from catkin_pkg.cmake import configure_file

data = configure_file(os.path.join(os.path.dirname(__file__), '..', '..', 'cmake', 'templates', '_setup_util.py.in'),
                      {
                          'CATKIN_GLOBAL_LIB_DESTINATION': 'lib',
                          'CATKIN_GLOBAL_BIN_DESTINATION': 'bin',
                          'PYTHON_INSTALL_DIR': 'pythonX.Y/packages',
                          'CMAKE_PREFIX_PATH_AS_IS': '',
                      })
with tempfile.TemporaryFile() as setup_util_file:
    setup_util_file.write(data)
    setup_util_file.seek(0)

    import imp
    imp.load_source('setup_util', '/somewhere/_setup_util.py', setup_util_file)

import setup_util
from setup_util import _get_workspaces, _prefix_env_variable, _rollback_env_variable, CATKIN_MARKER_FILE


class SetupUtilTest(unittest.TestCase):

    def test_get_reversed_workspaces(self):
        try:
            rootdir = tempfile.mkdtemp()
            mock_env = {}
            self.assertEqual([], _get_workspaces(mock_env))
            self.assertEqual([], _get_workspaces(mock_env, 'foo'))
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
            mock_env = {'CMAKE_PREFIX_PATH': foows}
            self.assertEqual([foows], _get_workspaces(mock_env))
            mock_env = {'CMAKE_PREFIX_PATH': os.pathsep.join([nows, foows, barws, 'invalid'])}
            self.assertEqual([foows, barws], _get_workspaces(mock_env))
        finally:
            shutil.rmtree(rootdir)

    def test_prefix_env(self):
        mock_env = {}
        self.assertEqual('',
                         _prefix_env_variable(mock_env, 'varname', [], ''))
        self.assertEqual(os.pathsep.join(['foo', 'bar']),
                         _prefix_env_variable(mock_env, 'varname', ['foo', 'bar'], ''))
        mock_env = {'varname': os.pathsep.join(['baz', 'bar', 'bam'])}
        self.assertEqual('',
                         _prefix_env_variable(mock_env, 'varname', [], ''))
        self.assertEqual('foo' + os.pathsep,
                         _prefix_env_variable(mock_env, 'varname', ['foo', 'bar'], ''))
        self.assertEqual(os.pathsep.join(['foo', 'lim']) + os.pathsep,
                         _prefix_env_variable(mock_env, 'varname', ['foo', 'lim', 'foo', 'lim'], ''))

    def test_remove_from_env(self):
        altsep = os.path.altsep
        try:
            rootdir = tempfile.mkdtemp()
            mock_env = {}
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
            # since workspace foo is not in CMAKE_PREFIX_PATH, it remains in varname
            self.assertEqual(foolib, _rollback_env_variable(mock_env, varname, '/lib'))

            # mock_env with both ws in CPP
            mock_env = {varname: os.pathsep.join([foolib, barlib]),
                        wsvarname: os.pathsep.join([foows, barws]),
                        'CMAKE_PREFIX_PATH': os.pathsep.join([foows, barws])}

            self.assertEqual(None, _rollback_env_variable(mock_env, varname, ''))
            self.assertEqual(None, _rollback_env_variable(mock_env, varname, 'nolib'))
            self.assertEqual(None, _rollback_env_variable(mock_env, varname, '/nolib'))
            self.assertEqual('', _rollback_env_variable(mock_env, varname, 'lib'))
            self.assertEqual('', _rollback_env_variable(mock_env, varname, '/lib'))
            self.assertEqual(None, _rollback_env_variable(mock_env, varname, ''))
            self.assertEqual('', _rollback_env_variable(mock_env, wsvarname, ''))

            # nows: not a workspace
            nows = os.path.join(rootdir, 'nows')
            nowslib = os.path.join(nows, 'lib')
            nowslib = os.path.join(nows, 'include')
            os.makedirs(nows)

            mock_env = {'varname': os.pathsep.join([foolib, nowslib, barlib, foolib]),
                        'CMAKE_PREFIX_PATH': os.pathsep.join([foows, barws])}
            # checks nows/lib remains, and second mention of foolib
            self.assertEqual(os.pathsep.join([nowslib, foolib]), _rollback_env_variable(mock_env, 'varname', '/lib'))
            self.assertEqual(os.pathsep.join([nowslib, foolib]), _rollback_env_variable(mock_env, 'varname', 'lib'))

            # windows pathsep
            os.path.altsep = '\\'
            self.assertEqual(os.pathsep.join([nowslib, foolib]), _rollback_env_variable(mock_env, 'varname', '\\lib'))
        finally:
            os.path.altsep = altsep
            shutil.rmtree(rootdir)
