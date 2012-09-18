import sys
import os
import unittest
import rospkg.stack
from mock import Mock, patch

try:
    from catkin.topological_order import _sort_projects, get_message_generators, \
        ProjectData, _topological_order_projects
except ImportError as impe:
    raise ImportError(
        'Please adjust your pythonpath before running this test: %s' % str(impe))


class TopoTest(unittest.TestCase):

    def test_project_data(self):
        mock_rospkg = Mock()
        mockproject = Mock()
        mockproject1 = Mock()
        mockproject2 = Mock()
        mockproject.build_depends = [mockproject1, mockproject2]
        with patch('rospkg.stack') as mock_rospkgstack:
            mock_rospkg.stack = mock_rospkgstack
            mock_rospkgstack.parse_stack_file.return_value = mockproject
            pd = ProjectData('foo/bar', rospkg_arg=mock_rospkg)
        self.assertEqual(pd.name, mockproject.name)
        self.assertEqual(pd.path, os.path.dirname('foo/bar'))
        self.assertEqual(pd.message_generator, mockproject.message_generator)
        self.assertEqual(pd.build_depends, set([mockproject1.name, mockproject2.name]))
        self.assertIsNotNone(str(pd))

    def test_sort_projects(self):
        projects = {}
        sprojects = _sort_projects(projects)
        self.assertEqual([], sprojects)

        mock1 = Mock()
        mock1.build_depends = set()
        mock1.message_generator = True

        mock2 = Mock()
        mock2.build_depends = set()
        mock2.message_generator = False

        mock3 = Mock()
        mock3.build_depends = Mock()
        mock3.message_generator = False

        projects = {'baz': mock3, 'bar': mock2, 'foo': mock1}
        sprojects = _sort_projects(projects)
        self.assertEqual([['foo', mock1], ['bar', mock2], [None, 'baz']], sprojects)

    def test_get_message_generators(self):

        projects = []
        gens = get_message_generators(projects)
        self.assertEqual([], gens)

        mock1 = Mock()
        mock1.message_generator = True
        mock2 = Mock()
        mock2.message_generator = False

        projects = [('foo', mock1), ('bar', mock2)]
        gens = get_message_generators(projects)
        self.assertEqual(['foo'], gens)

    def test_topological_order_projects(self):
        plist = []
        self.assertEqual([], _topological_order_projects(plist))

        p1 = Mock('catkin.topological_order.ProjectData')
        p1.name = 'p1'
        p1.build_depends = set(['p3', 'px'])
        p1.message_generator = Mock()

        p2 = Mock('catkin.topological_order.ProjectData')
        p2.name = 'p2'
        p2.build_depends = set(['p1', 'p3'])
        p2.message_generator = Mock()

        p3 = Mock('catkin.topological_order.ProjectData')
        p3.name = 'p3'
        p3.build_depends = set()
        p3.message_generator = Mock()

        p4 = Mock('catkin.topological_order.ProjectData')
        p4.name = 'catkin'

        plist = [p1, p2, p3]
        self.assertEqual([['p3', p3], ['p1', p1], ['p2', p2]],
                         _topological_order_projects(plist))
