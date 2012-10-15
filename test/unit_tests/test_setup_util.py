import os
import unittest
import tempfile
import shutil

import imp
imp.load_source('setup_util',
                os.path.join(os.path.dirname(__file__),
                             '..', '..', 'cmake', 'templates', 'setup.py'))

import setup_util
from setup_util import get_reversed_workspaces, CATKIN_WORKSPACE_MARKER_FILE


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

