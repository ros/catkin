import sys
import os
from os.path import join
import unittest
import tempfile
import shutil

from mock import MagicMock

from catkin.templates import instantiate_templates, create_files

class TemplateTest(unittest.TestCase):

    def test_create_files(self):
        rootdir = tempfile.mkdtemp()
        file1 = os.path.join('foo', 'bar')
        file2 = os.path.join('foo', 'baz')
        newfiles = {file1: 'foobar', file2: 'barfoo'}
        try:
            create_files(newfiles, rootdir)
            self.assertTrue(os.path.isfile(os.path.join(rootdir, file1)))
            self.assertTrue(os.path.isfile(os.path.join(rootdir, file2)))
            self.assertRaises(ValueError, create_files, newfiles, rootdir)
        finally:
            shutil.rmtree(rootdir)
        

    def test_instantiate_package(self):
        templates_source_dir = os.path.join(os.path.dirname(__file__),
                                            '..', '..', 'templates', 'package')
        mock_attrs = MagicMock()
        globaldict = {'CATKIN_PACKAGE': mock_attrs}
        newfiles = instantiate_templates(templates_source_dir,
                                         globaldict,
                                         unittest=True)
        self.assertTrue('CMakeLists.txt' in newfiles)
        self.assertTrue('manifest.xml' in newfiles)
