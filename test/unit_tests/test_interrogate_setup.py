import os
import unittest
import tempfile
import shutil

import imp
imp.load_source('interrogate_setup_dot_py',
                os.path.join(os.path.dirname(__file__),
                             '..', '..', 'cmake', 'interrogate_setup_dot_py.py'))

from interrogate_setup_dot_py import Dummy, generate_cmake_file


class InterrogateSetupTest(unittest.TestCase):

    def test_generate_cmake_file(self):
        cmake_lines = (generate_cmake_file(package_name='pack1',
                                           version='0.0.1',
                                           scripts=[],
                                           package_dir={'': 'foopath'},
                                           pkgs=['foo', 'bar', 'bar.sub']))
        self.assertEqual(['set(pack1_VERSION "0.0.1")',
                          'set(pack1_SCRIPTS "")',
                          'set(pack1_PACKAGES "foo;bar")',
                          'set(pack1_PACKAGE_DIRS "foopath/foo;foopath/bar")']
, cmake_lines)




    def test_interrogate_setup_py(self):
        try:
            rootdir = tempfile.mkdtemp()
            outfile = os.path.join(rootdir, 'out.cmake')
            dummy = Dummy('foo', outfile)
            self.assertRaises(RuntimeError, dummy.setup, version='0.1.1')
            self.assertRaises(RuntimeError, dummy.setup, package_dir={'': 'src'})
        finally:
            shutil.rmtree(rootdir)
