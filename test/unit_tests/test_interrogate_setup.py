import os
import unittest
import tempfile
import shutil

import imp
imp.load_source('interrogate_setup_dot_py',
                os.path.join(os.path.dirname(__file__),
                             '..', '..', 'cmake', 'interrogate_setup_dot_py.py'))

from interrogate_setup_dot_py import Dummy, generate_cmake_file, _get_locations


class InterrogateSetupTest(unittest.TestCase):

    def test_generate_cmake_file(self):
        cmake_lines = (generate_cmake_file(package_name='pack1',
                                           version='0.0.1',
                                           scripts=[],
                                           package_dir={'': 'foopath'},
                                           pkgs=['foo', 'bar', 'bar.sub']))
        self.assertEqual(['set(pack1_SETUP_PY_VERSION "0.0.1")',
                          'set(pack1_SETUP_PY_SCRIPTS "")',
                          'set(pack1_SETUP_PY_PACKAGES "foo;bar")',
                          'set(pack1_SETUP_PY_PACKAGE_DIRS "foopath/foo;foopath/bar")'],
                         cmake_lines)
        cmake_lines = (generate_cmake_file(package_name='pack1',
                                           version='0.0.1',
                                           scripts=[],
                                           package_dir={},
                                           pkgs=['foo', 'bar', 'bar.sub']))
        self.assertEqual(['set(pack1_SETUP_PY_VERSION "0.0.1")',
                          'set(pack1_SETUP_PY_SCRIPTS "")',
                          'set(pack1_SETUP_PY_PACKAGES "foo;bar")',
                          'set(pack1_SETUP_PY_PACKAGE_DIRS "foo;bar")'],
                         cmake_lines)
        cmake_lines = (generate_cmake_file(package_name='pack1',
                                           version='0.0.1',
                                           scripts=['bin/foo', 'nodes/bar'],
                                           package_dir={},
                                           pkgs=['foo', 'bar', 'bar.sub']))
        self.assertEqual(['set(pack1_SETUP_PY_VERSION "0.0.1")',
                          'set(pack1_SETUP_PY_SCRIPTS "bin/foo;nodes/bar")',
                          'set(pack1_SETUP_PY_PACKAGES "foo;bar")',
                          'set(pack1_SETUP_PY_PACKAGE_DIRS "foo;bar")'],
                         cmake_lines)

    def test_get_locations(self):
        self.assertEqual({'foo': ''}, _get_locations(['foo'], {}))
        self.assertEqual({'foo': 'src'}, _get_locations(['foo'],
                                                        {'': 'src'}))
        self.assertEqual({'foo': 'src', 'foo.bar': 'src'}, _get_locations(['foo.bar'],
                                                                          {'': 'src'}))
        self.assertEqual({'foo': 'src'}, _get_locations(['foo'],
                                                        {'foo': 'src'}))
        self.assertEqual({'foo': 'src', 'foo.bar': 'src'}, _get_locations(['foo.bar'],
                                                                          {'foo': 'src'}))

    def test_generate_cmake_file_noallprefix(self):
        cmake_lines = (generate_cmake_file(package_name='pack1',
                                           version='0.0.1',
                                           scripts=[],
                                           package_dir={'foo': 'src',
                                                        'bar': 'lib'},
                                           pkgs=['foo', 'bar', 'bar.sub']))
        self.assertEqual(['set(pack1_SETUP_PY_VERSION "0.0.1")',
                          'set(pack1_SETUP_PY_SCRIPTS "")',
                          'set(pack1_SETUP_PY_PACKAGES "foo;bar")',
                          'set(pack1_SETUP_PY_PACKAGE_DIRS "src/foo;lib/bar")'],
                         cmake_lines)

    def test_generate_cmake_file_msg_srv(self):
        cmake_lines = (generate_cmake_file(package_name='pack1',
                                           version='0.0.1',
                                           scripts=[],
                                           package_dir={'foo.msg': 'msg',
                                                        'foo.srv': 'srv',
                                                        '': 'src'},
                                           pkgs=['foo.msg', 'foo.srv', 'foo']))
        self.assertEqual(['set(pack1_SETUP_PY_VERSION "0.0.1")',
                          'set(pack1_SETUP_PY_SCRIPTS "")',
                          'set(pack1_SETUP_PY_PACKAGES "foo")',
                          'set(pack1_SETUP_PY_PACKAGE_DIRS "src/foo")'],
                         cmake_lines)

    def test_generate_cmake_file_invalid(self):
        self.assertRaises(RuntimeError,
                          generate_cmake_file,
                          package_name='pack1',
                          version='0.0.1',
                          scripts=[],
                          package_dir={'foo.sub1': 'sub1',
                                       'foo.sub2': 'somewhere',
                                       '': 'src'},
                          pkgs=['foo.sub2', 'foo.sub1', 'foo'])

    def test_interrogate_setup_py(self):
        try:
            rootdir = tempfile.mkdtemp()
            outfile = os.path.join(rootdir, 'out.cmake')
            dummy = Dummy('foo', outfile)
            self.assertRaises(RuntimeError, dummy.setup, version='0.1.1')
            self.assertRaises(RuntimeError, dummy.setup, package_dir={'': 'src'})
            # simple setup
            dummy.setup(version='0.1.1', package_dir={'': 'src'})
            self.assertTrue(os.path.isfile(outfile))
            with open(outfile, 'r') as fhand:
                contents = fhand.read()
            self.assertEqual("""set(foo_SETUP_PY_VERSION "0.1.1")
set(foo_SETUP_PY_SCRIPTS "")
set(foo_SETUP_PY_PACKAGES "")
set(foo_SETUP_PY_PACKAGE_DIRS "")""", contents)
            os.remove(outfile)
            # packages and scripts
            dummy.setup(version='0.1.1', package_dir={}, packages=['foo', 'bar'], scripts=['bin/foo', 'nodes/bar'])
            self.assertTrue(os.path.isfile(outfile))
            with open(outfile, 'r') as fhand:
                contents = fhand.read()
            self.assertEqual("""set(foo_SETUP_PY_VERSION "0.1.1")
set(foo_SETUP_PY_SCRIPTS "bin/foo;nodes/bar")
set(foo_SETUP_PY_PACKAGES "foo;bar")
set(foo_SETUP_PY_PACKAGE_DIRS "foo;bar")""", contents)
            os.remove(outfile)
            # packages and package_dir
            dummy.setup(version='0.1.1', package_dir={'foo': 'src', 'bar': 'lib'}, packages=['foo', 'bar'],)
            self.assertTrue(os.path.isfile(outfile))
            with open(outfile, 'r') as fhand:
                contents = fhand.read()
            self.assertEqual("""set(foo_SETUP_PY_VERSION "0.1.1")
set(foo_SETUP_PY_SCRIPTS "")
set(foo_SETUP_PY_PACKAGES "foo;bar")
set(foo_SETUP_PY_PACKAGE_DIRS "src/foo;lib/bar")""", contents)
            os.remove(outfile)
        finally:
            shutil.rmtree(rootdir)
