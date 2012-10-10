import os
import unittest
import tempfile
import shutil
from mock import Mock

try:
    import catkin.find_in_workspaces
    from catkin.find_in_workspaces import find_in_workspaces, _get_valid_search_dirs
    from catkin.workspace import CATKIN_WORKSPACE_MARKER_FILE
except ImportError as impe:
    raise ImportError(
        'Please adjust your pythonpath before running this test: %s' % str(impe))


class FindInWorkspaceTest(unittest.TestCase):

    def test_get_valid_search_dirs(self):
        self.assertEqual(['bin', 'etc', 'include', 'lib', 'share'], _get_valid_search_dirs([], None))
        self.assertEqual(['bin', 'etc', 'include', 'lib', 'share'], _get_valid_search_dirs(None, None))
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
        ## invalid tests case, search with path with no project given
        # (existing, paths) = find_in_workspaces(['share', 'etc', 'lib'], None, 'foopath', _workspaces=['bar', 'baz'])
        # self.assertEqual(([], ['bar/share/foopath',
        #                        'bar/etc/foopath',
        #                        'bar/lib/foopath',
        #                        'baz/share/foopath',
        #                        'baz/etc/foopath',
        #                        'baz/lib/foopath']), (existing, paths))
        self.assertRaises(RuntimeError, find_in_workspaces, None, None, 'foopath')
        (existing, paths) = find_in_workspaces(None, None, None, _workspaces=['bar'])
        self.assertEqual(([], ['bar/bin', 'bar/etc', 'bar/include', 'bar/lib', 'bar/share']), (existing, paths))


    def test_with_sourcepath(self):
        def create_mock_workspace(root_dir, ws):
            ws = os.path.join(root_dir, ws)
            bs = os.path.join(ws, 'buildspace')
            src = os.path.join(ws, 'src')
            inc = os.path.join(bs, "include")
            etc = os.path.join(bs, "etc")
            p1inc = os.path.join(inc, "p1")
            p1etc = os.path.join(etc, "p1")
            p1src = os.path.join(src, "p1")

            os.makedirs(ws)
            os.makedirs(bs)
            os.makedirs(inc)
            os.makedirs(etc)
            os.makedirs(src)
            os.makedirs(p1src)
            os.makedirs(p1inc)
            os.makedirs(p1etc)

            with open(os.path.join(bs, CATKIN_WORKSPACE_MARKER_FILE), 'w') as fhand:
                fhand.write(src)
        try:
            root_dir = tempfile.mkdtemp()
            # monkey-patch catkin_pkg
            backup = catkin.find_in_workspaces.find_packages
            mock1 = Mock()
            mock1.name = 'p1'
            def mock_find_packages(basepath):
                return {'p1': mock1}
            catkin.find_in_workspaces.find_packages = mock_find_packages

            create_mock_workspace(root_dir, 'ws1')
            create_mock_workspace(root_dir, 'ws2')

            (existing, paths) = find_in_workspaces(['include', 'etc'], 'foo', 'foopath', _workspaces=[os.path.join(root_dir, 'ws1/buildspace')])
            self.assertEqual([os.path.join(root_dir, 'ws1', 'buildspace', 'include', 'foo', 'foopath'),
                              os.path.join(root_dir, 'ws1', 'buildspace', 'etc', 'foo', 'foopath')], paths)
            self.assertEqual([], existing)

            (existing, paths) = find_in_workspaces(['share', 'etc'], 'p1', None, _workspaces=[os.path.join(root_dir, 'ws1/buildspace')])
            self.assertEqual([os.path.join(root_dir, 'ws1', 'buildspace', 'share', 'p1'),
                              # share means also search in src
                              os.path.join(root_dir, 'ws1', 'src', 'p1'),
                              os.path.join(root_dir, 'ws1', 'buildspace', 'etc', 'p1')], paths)
            self.assertEqual([os.path.join(root_dir, 'ws1', 'src', 'p1'), os.path.join(root_dir, 'ws1', 'buildspace', 'etc', 'p1')], existing)

            ## OVERLAY ws2 > ws1
            (existing, paths) = find_in_workspaces(['share', 'etc'], 'p1', None, _workspaces=[os.path.join(root_dir, 'ws2/buildspace'), os.path.join(root_dir, 'ws1/buildspace')])
            self.assertEqual([os.path.join(root_dir, 'ws2', 'buildspace', 'share', 'p1'),
                              # share means also search in src
                              os.path.join(root_dir, 'ws2', 'src', 'p1'),
                              os.path.join(root_dir, 'ws2', 'buildspace', 'etc', 'p1')], paths)
            self.assertEqual([os.path.join(root_dir, 'ws2', 'src', 'p1'), os.path.join(root_dir, 'ws2', 'buildspace', 'etc', 'p1')], existing)

            (existing, paths) = find_in_workspaces([], 'p1', None, _workspaces=[os.path.join(root_dir, 'ws2/buildspace'), os.path.join(root_dir, 'ws1/buildspace')])
            self.assertEqual([os.path.join(root_dir, 'ws2', 'buildspace', 'etc', 'p1'),
                              os.path.join(root_dir, 'ws2', 'buildspace', 'include', 'p1'),
                              os.path.join(root_dir, 'ws2', 'buildspace', 'lib', 'p1'),
                              os.path.join(root_dir, 'ws2', 'buildspace', 'share', 'p1'),

                              # share means also search in src
                              os.path.join(root_dir, 'ws2', 'src', 'p1')], paths)
            self.assertEqual([os.path.join(root_dir, 'ws2', 'buildspace', 'etc', 'p1'), os.path.join(root_dir, 'ws2', 'buildspace', 'include', 'p1'), os.path.join(root_dir, 'ws2', 'src', 'p1')], existing)
            
        finally:
            shutil.rmtree(root_dir)
            catkin.find_in_workspaces.find_packages = backup
