import os
import unittest
import tempfile
import shutil
import mock

try:
    import catkin.builder
    from catkin.builder import build_workspace_in_isolation
except ImportError as impe:
    raise ImportError(
        'Please adjust your pythonpath before running this test: %s' % str(impe))


class BuilderTest(unittest.TestCase):

    def test_build_workspace_in_isolation(self):
        backup_top = catkin.builder.topological_order
        backup_run = catkin.builder._run_command_with_env
        old_cwd = os.getcwd()
        try:
            root_dir = tempfile.mkdtemp()
            os.chdir(root_dir)
            mockp1 = mock.Mock()
            mockp1.exports = []
            mockp1.name = 'p1'
            mockp1.filename = '/foo/package.xml'
            mockp2 = mock.Mock()
            mockp2.exports = []
            mockp2.name = 'p2'
            mockp2.filename = '/bar/package.xml'
            mock_top = mock.Mock()
            mock_top.return_value = [('foopack', mockp1), ('barpack', mockp2)]
            mock_run = mock.Mock()
            catkin.builder.topological_order = mock_top
            catkin.builder._run_command_with_env = mock_run
            build_workspace_in_isolation('/path1')
            self.assertEqual(1, mock_top.call_count)
            top_args = mock_top.call_args[0]
            self.assertEqual(('/path1',), top_args)
            self.assertEqual(4, mock_run.call_count, mock_run.call_args_list)
            run_args = mock_run.call_args_list
            self.assertEqual(['cmake', '/foo', '-DCATKIN_DEVEL_PREFIX=%s' % os.path.join(root_dir, 'p1__devel')], run_args[0][0][0])
            self.assertEqual(['make', '-j8'], run_args[1][0][0])
            # install case
            mock_top.reset_mock()
            mock_run.reset_mock()
            build_workspace_in_isolation('/path1', install=True)
            self.assertEqual(1, mock_top.call_count)
            top_args = mock_top.call_args[0]
            self.assertEqual(('/path1',), top_args)
            self.assertEqual(6, mock_run.call_count)
            run_args = mock_run.call_args_list
            self.assertEqual(['cmake', '/foo', '-DCATKIN_DEVEL_PREFIX=%s' % os.path.join(root_dir, 'p1__devel'), '-DCMAKE_INSTALL_PREFIX=%s' % os.path.join(root_dir, 'p1__install')], run_args[0][0][0])
            self.assertEqual(['make', '-j8'], run_args[1][0][0])
            self.assertEqual(['make', 'install'], run_args[2][0][0])
        finally:
            os.chdir(old_cwd)
            catkin.builder.topological_order = backup_top
            catkin.builder._run_command_with_env = backup_run
            shutil.rmtree(root_dir)
