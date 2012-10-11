import os
import unittest

from mock import Mock

import imp
imp.load_source('create_plugin_xml',
                os.path.join(os.path.dirname(__file__),
                             '..', '..', 'cmake', 'create_plugin_xml.py'))

from create_plugin_xml import _get_output


class PluginXmlTest(unittest.TestCase):

    def test_get_output(self):
        package = Mock()
        package.exports = []
        package.depends = []
        result = _get_output(package, True, False)
        self.assertEqual([], result)
        e1 = Mock(attributes={'rosfoo': 'bar'})
        e2 = Mock(tagname='e2', attributes={'rosfoo': 'bar', 'plugin': 'mockplug1'})
        e2.name = 'e2'
        d1 = Mock(attributes={'rosfoo': 'bar'})
        d2 = Mock(tagname='d2', attributes={'rosfoo': 'bar', 'plugin': 'mockplug2'})
        package.exports = [e1, e2]
        package.depends = [d1, d2]
        result = _get_output(package, False, True)
        self.assertEqual(['<depend package="e2"/>'], result)
