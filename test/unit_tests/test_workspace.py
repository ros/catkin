import os
import unittest
import tempfile
import shutil

try:
    from catkin.workspace import get_workspaces, get_source_paths

    from catkin.workspace import CATKIN_WORKSPACE_MARKER_FILE
except ImportError as impe:
    raise ImportError(
        'Please adjust your pythonpath before running this test: %s' % str(impe))


class WorkspaceTest(unittest.TestCase):

    def test_get_workspaces(self):
        try:
            root_dir = tempfile.mkdtemp()
            ws1 = os.path.join(root_dir, 'ws1')
            ws2 = os.path.join(root_dir, 'ws2')
            os.makedirs(ws1)
            os.makedirs(ws2)
            with open(os.path.join(ws1, CATKIN_WORKSPACE_MARKER_FILE), 'w') as fhand:
                fhand.write('loc1;loc2')
            with open(os.path.join(ws2, CATKIN_WORKSPACE_MARKER_FILE), 'w') as fhand:
                fhand.write('loc3;loc4')
            self.assertEqual([], get_workspaces(_environ={}))
            self.assertEqual([], get_workspaces(_environ={'CMAKE_PREFIX_PATH': ''}))
            self.assertEqual([], get_workspaces(_environ={}))
            self.assertEqual([ws1], get_workspaces(_environ={'CMAKE_PREFIX_PATH': ws1}))
            self.assertEqual([], get_workspaces(_environ={}))
            self.assertEqual([], get_workspaces(_environ={'CMAKE_PREFIX_PATH': 'nowhere'}))
            self.assertEqual([ws2, ws1], get_workspaces(_environ={'CMAKE_PREFIX_PATH': ws2 + os.pathsep + ws1}))
        finally:
            shutil.rmtree(root_dir)

    def test_get_source_paths(self):
        try:
            root_dir = tempfile.mkdtemp()
            ws1 = os.path.join(root_dir, 'ws1')
            ws2 = os.path.join(root_dir, 'ws2')
            os.makedirs(ws1)
            os.makedirs(ws2)
            with open(os.path.join(ws1, CATKIN_WORKSPACE_MARKER_FILE), 'w') as fhand:
                fhand.write('loc1;loc2')
            with open(os.path.join(ws2, CATKIN_WORKSPACE_MARKER_FILE), 'w') as fhand:
                fhand.write('')
            self.assertEqual(['loc1', 'loc2'], get_source_paths(ws1))
            self.assertEqual([], get_source_paths(ws2))
            self.assertRaises(ValueError, get_source_paths, root_dir)
        finally:
            shutil.rmtree(root_dir)
