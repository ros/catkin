import os
import unittest
import tempfile
import shutil

try:
    from catkin.find_in_workspaces import find_in_workspaces, _get_valid_search_dirs
    from catkin.workspace import CATKIN_WORKSPACE_MARKER_FILE
except ImportError as impe:
    raise ImportError(
        'Please adjust your pythonpath before running this test: %s' % str(impe))


class FindInWorkspaceTest(unittest.TestCase):

    def test_get_valid_search_dirs(self):
        self.assertEqual([], _get_valid_search_dirs([], None))
        self.assertEqual(['etc', 'include', 'libexec', 'share'],
                         _get_valid_search_dirs(None, 'foo'))
        self.assertEqual(['bin', 'etc', 'include', 'lib', 'share'],
                         _get_valid_search_dirs(None, None))
        self.assertEqual(['include', 'bin'],
                         _get_valid_search_dirs(['include', 'bin'], None))
        self.assertEqual(['include', 'etc'],
                         _get_valid_search_dirs(['include', 'etc'], 'foo'))

        self.assertRaises(ValueError, _get_valid_search_dirs, ['foo'], None)
        self.assertRaises(ValueError, _get_valid_search_dirs, ['bin'], 'foo')
        self.assertRaises(ValueError, _get_valid_search_dirs, ['libexec'], None)

    def test_find_in_workspaces(self):
        (existing, paths) = find_in_workspaces([], _workspaces=None)
        self.assertEqual(([], []), (existing, paths))
        (existing, paths) = find_in_workspaces([], 'foo', _workspaces=None)
        self.assertEqual(([], []), (existing, paths))
        (existing, paths) = find_in_workspaces([], 'foo', 'foopath', _workspaces=None)
        self.assertEqual(([], []), (existing, paths))

        (existing, paths) = find_in_workspaces(['include'], 'foo', 'foopath', _workspaces=None)
        self.assertEqual(([], []), (existing, paths))

        (existing, paths) = find_in_workspaces(['include'], 'foo', 'foopath', _workspaces=['bar'])
        self.assertEqual(([], ['bar/include/foo/foopath']), (existing, paths))

        (existing, paths) = find_in_workspaces(['include'], 'foo', 'foopath', _workspaces=['bar', 'baz'])
        self.assertEqual(([], ['bar/include/foo/foopath', 'baz/include/foo/foopath']), (existing, paths))

        (existing, paths) = find_in_workspaces(['include', 'etc', 'libexec'], 'foo', 'foopath', _workspaces=['bar', 'baz'])
        self.assertEqual(([], ['bar/include/foo/foopath',
                               'bar/etc/foo/foopath',
                               'bar/lib/foo/foopath',
                               'baz/include/foo/foopath',
                               'baz/etc/foo/foopath',
                               'baz/lib/foo/foopath']), (existing, paths))

        (existing, paths) = find_in_workspaces(['share', 'etc', 'lib'], None, 'foopath', _workspaces=['bar', 'baz'])
        self.assertEqual(([], ['bar/share/foopath',
                               'bar/etc/foopath',
                               'bar/lib/foopath',
                               'baz/share/foopath',
                               'baz/etc/foopath',
                               'baz/lib/foopath']), (existing, paths))
        (existing, paths) = find_in_workspaces(None, None, None, _workspaces=['bar'])
        self.assertEqual(([], ['bar/bin', 'bar/etc', 'bar/include', 'bar/lib', 'bar/share']), (existing, paths))


    def test_with_sourcepath(self):
        def create_mock_workspace(root_dir, ws):
            ws1 = os.path.join(root_dir, ws)
            p1 = os.path.join(ws1, "p1")
            p1inc = os.path.join(p1, "include")
            p1share = os.path.join(p1, "share")
            os.makedirs(ws1)
            os.makedirs(p1)
            os.makedirs(p1inc)
            os.makedirs(p1share)
            with open(os.path.join(ws1, CATKIN_WORKSPACE_MARKER_FILE), 'w') as fhand:
                fhand.write('loc1;loc2')

        try:
            root_dir = tempfile.mkdtemp()
            create_mock_workspace(root_dir, 'ws1')
            create_mock_workspace(root_dir, 'ws2')

            (existing, paths) = find_in_workspaces(['share', 'etc'], 'foo', 'foopath', _workspaces=[os.path.join(root_dir, 'ws1')])
            self.assertEqual([os.path.join(root_dir, 'ws1', 'share', 'foo', 'foopath'),
                              'loc1/foo/foopath',
                              'loc2/foo/foopath',
                              os.path.join(root_dir, 'ws1', 'etc', 'foo', 'foopath')], paths)
            self.assertEqual([], existing)

            (existing, paths) = find_in_workspaces(['share', 'etc'], 'p1', None, _workspaces=[os.path.join(root_dir, 'ws1')])
            self.assertEqual([os.path.join(root_dir, 'ws1', 'share', 'p1'),
                              'loc1/p1',
                              'loc2/p1',
                              os.path.join(root_dir, 'ws1', 'etc', 'p1')], paths)
            self.assertEqual([], existing)

        finally:
            shutil.rmtree(root_dir)
