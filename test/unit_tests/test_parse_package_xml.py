import os
import unittest
import tempfile
import shutil
from mock import Mock

import imp
imp.load_source('parse_package_xml',
                os.path.join(os.path.dirname(__file__),
                             '..', '..', 'cmake', 'parse_package_xml.py'))

from parse_package_xml import _get_output, main


class ParsePackageXmlTest(unittest.TestCase):

    def test_get_output(self):
        pack = Mock(maintainers=['m1', 'm2'], build_depends=['bd1', 'bd2'], run_depends=['rd1', 'rd2'], version='0.1.2')
        # mock name param is different
        pack.name = 'foopack'
        result = _get_output(pack)
        self.assertEqual(['set(_CATKIN_CURRENT_PACKAGE "foopack")',
                          'set(foopack_RUN_DEPENDS "rd1" "rd2")',
                          'set(foopack_VERSION "0.1.2")',
                          'set(foopack_MAINTAINER "m1, m2")',
                          'set(foopack_BUILD_DEPENDS "bd1" "bd2")'], result)

    def test_main(self):
        try:
            rootdir = tempfile.mkdtemp()
            src_file = os.path.join(rootdir, 'package.xml')
            check_file = os.path.join(rootdir, 'foo.cmake')
            with open(src_file, 'w') as fhand:
                fhand.write('''<package>
<name>foopack</name>
<version>0.1.2</version>
<description>foo</description>
<maintainer email='foo@bar.com'>foo</maintainer>
<license>foo</license>
<run_depend>rd1</run_depend>
<run_depend>rd2</run_depend>
<build_depend>bd1</build_depend>
<build_depend>bd2</build_depend>
</package>''')
            main([src_file, check_file])
            self.assertTrue(os.path.isfile(check_file))
            with open(check_file, 'r') as fhand:
                contents = fhand.read()
            self.assertEqual('''set(_CATKIN_CURRENT_PACKAGE "foopack")
set(foopack_RUN_DEPENDS "rd1" "rd2")
set(foopack_VERSION "0.1.2")
set(foopack_MAINTAINER "foo <foo@bar.com>")
set(foopack_BUILD_DEPENDS "bd1" "bd2")''', contents)
        finally:
            shutil.rmtree(rootdir)
