import sys
import os
import unittest
import rospkg.stack
from mock import Mock, patch

try:
    from catkin.topological_order import _sort_packages, \
        get_message_generators, PackageData, _topological_order_packages
except ImportError as impe:
    raise ImportError(
        'Please adjust your pythonpath before running this test: %s' % str(impe))


class TopoTest(unittest.TestCase):

    def test_package_data_init(self):

        mockproject = Mock()
        mockproject1 = Mock()
        mockproject2 = Mock()
        mockproject3 = Mock()
        mockproject4 = Mock()
        mockexport = Mock()
        mockexport.tagname = 'message_generator'
        mockexport.content = 'foolang'
        mockproject.build_depends = [mockproject1, mockproject2]
        mockproject.buildtool_depends = [mockproject3, mockproject4]
        mockproject.exports = [mockexport]

        def mock_parse_package_arg(path):
            return mockproject
        pd = PackageData('foo/bar', parse_package_arg=mock_parse_package_arg)
        self.assertEqual(mockproject.name, pd.name)
        self.assertEqual('foo/bar', pd.path)
        self.assertEqual(mockexport.content, pd.message_generator)
        self.assertEqual(set([mockproject1.name, mockproject2.name, mockproject3.name, mockproject4.name]), pd.build_depends)
        self.assertIsNotNone(str(pd))

    def test_sort_packages(self):
        projects = {}
        sprojects = _sort_packages(projects)
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
        sprojects = _sort_packages(projects)
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

    def test_topological_order_packages(self):
        plist = []
        self.assertEqual([], _topological_order_packages(plist))

        p1 = Mock('catkin.topological_order.PackageData')
        p1.name = 'p1'
        p1.build_depends = set(['p3', 'px'])
        p1.message_generator = Mock()

        p2 = Mock('catkin.topological_order.PackageData')
        p2.name = 'p2'
        p2.build_depends = set(['p1', 'p3'])
        p2.message_generator = Mock()

        p3 = Mock('catkin.topological_order.PackageData')
        p3.name = 'p3'
        p3.build_depends = set()
        p3.message_generator = Mock()

        p4 = Mock('catkin.topological_order.PackageData')
        p4.name = 'catkin'

        plist = [p1, p2, p3]
        self.assertEqual([['p3', p3], ['p1', p1], ['p2', p2]],
                         _topological_order_packages(plist))
