import sys
import os
import unittest
from catkin.topological_order import _sort_projects, get_message_generators
from mock import Mock

class TopoTest(unittest.TestCase):

    def test_sort_projects(self):
        projects = {}
        sprojects = _sort_projects(projects)
        self.assertEqual([], sprojects)

        mock1 = Mock()
        mock1.build_depends = None
        mock1.message_generator = True

        mock2 = Mock()
        mock2.build_depends = None
        mock2.message_generator = False

        mock3 = Mock()
        mock3.build_depends = Mock()
        mock3.message_generator = False

        projects = {'baz': mock3,  'bar': mock2, 'foo': mock1}
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
