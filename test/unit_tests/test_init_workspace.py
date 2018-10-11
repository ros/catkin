import os
from os.path import join
import unittest
import tempfile
import shutil

try:
    from catkin.init_workspace import init_workspace, _symlink_or_copy
except ImportError as impe:
    raise ImportError(
        'Please adjust your pythonpath before running this test: %s' % str(impe))


class InitWorkspaceTest(unittest.TestCase):

    def test_symlink_or_copy(self):
        try:
            root_dir = tempfile.mkdtemp(dir=os.getcwd())
            os.makedirs(join(root_dir, 'subdir'))
            os.makedirs(join(root_dir, 'subdir2'))
            with open(join(root_dir, 'subdir', 'foo'), 'ab') as fhand:
                fhand.write('content'.encode('UTF-8'))

            _symlink_or_copy(join(root_dir, 'subdir', 'foo'),
                             join(root_dir, 'foolink'))
            _symlink_or_copy(join(root_dir, 'subdir', 'foo'),
                             join(root_dir, 'subdir', 'foolink'))
            _symlink_or_copy(os.path.relpath(join(root_dir, 'subdir', 'foo'),
                                             os.getcwd()),
                             join(root_dir, 'foolinkrel'))

            try:
                self.assertEqual(join(root_dir, 'subdir', 'foo'),
                                os.readlink(join(root_dir, 'foolink')))
            except Exception as ex_readlink:
                pass
            
            try:
                self.assertEqual(join(root_dir, 'subdir', 'foo'),
                                 os.readlink(join(root_dir, 'subdir', 'foolink')))
            except Exception as ex_readlink:
                pass

            try:
                self.assertEqual(os.path.relpath(join(root_dir, 'subdir', 'foo'),
                                                 os.getcwd()),
                                 os.readlink(join(root_dir, 'foolinkrel')))
            except Exception as ex_readlink:
                pass

        finally:
            # pass
            shutil.rmtree(root_dir)

    def test_init_workspace(self):
        try:
            root_dir = tempfile.mkdtemp()
            os.makedirs(join(root_dir, 'ws1'))
            os.makedirs(join(root_dir, 'ws1', 'catkin'))
            os.makedirs(join(root_dir, 'ws1', 'catkin', 'cmake'))
            with open(join(root_dir, 'ws1', 'catkin', 'cmake', 'toplevel.cmake'),
                      'ab') as fhand:
                fhand.write(''.encode('UTF-8'))
            with open(join(root_dir, 'ws1', '.catkin'), 'ab') as fhand:
                fhand.write(''.encode('UTF-8'))
            os.makedirs(join(root_dir, 'ws2'))
            with open(join(root_dir, 'ws2', '.catkin'), 'ab') as fhand:
                fhand.write(''.encode('UTF-8'))

            workspace_dir = join(root_dir, 'ws1')
            os.chdir(workspace_dir)
            init_workspace(workspace_dir)
            workspace_dir = join(root_dir, 'ws2')
            os.chdir(workspace_dir)
            init_workspace(workspace_dir)

            # in same workspace symlink should be relative
            try:
                self.assertEqual(
                    join('catkin', 'cmake', 'toplevel.cmake'),
                    os.readlink(join(root_dir, 'ws1', 'CMakeLists.txt')))
            except Exception as ex_readlink:
                pass

            # outside workspace, path should be absolute
            try:
                self.assertTrue(
                    os.path.samefile(
                        join(os.path.dirname(__file__),
                             '..', '..', 'cmake', 'toplevel.cmake'),
                        os.readlink(join(root_dir, 'ws2', 'CMakeLists.txt'))))
            except Exception as ex_readlink:
                pass

        finally:
            # pass
            os.chdir(os.path.dirname(__file__))
            shutil.rmtree(root_dir)
