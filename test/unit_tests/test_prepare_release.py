import os
import unittest

import imp
imp.load_source('catkin_prepare_release',
                os.path.join(os.path.dirname(__file__),
                             '..', '..', 'bin', 'catkin_prepare_release'))

from catkin_prepare_release import get_commands


class CatkinPrepareReleaseTest(unittest.TestCase):

    def test_get_commands(self):
        commands = get_commands('foo', {}, 'git', '0.1.2', svn_url=None)
        self.assertEqual(['bin/catkin_package_version --bump patch foo',
                          'git commit -m "0.1.2" ',
                          'git push',
                          'git tag 0.1.2',
                          'git push --tags'], commands)

        commands = get_commands('foo', {'pack1': 'bar'}, 'git', '0.1.2', svn_url=None)
        self.assertEqual(['bin/catkin_package_version --bump patch foo',
                          'git commit -m "0.1.2" pack1/package.xml',
                          'git push',
                          'git tag 0.1.2',
                          'git push --tags'], commands)

        commands = get_commands('foo', {'pack1': 'bar', 'pack2': 'foo'}, 'git', '0.1.2', svn_url=None)
        self.assertEqual(['bin/catkin_package_version --bump patch foo',
                          'git commit -m "0.1.2" pack1/package.xml pack2/package.xml',
                          'git push',
                          'git tag 0.1.2',
                          'git push --tags'], commands)

    def test_get_commands_svn(self):
        commands = get_commands('foo', {}, 'svn', '0.1.2', svn_url='http://foo.bar/project1/trunk')
        self.assertEqual(['bin/catkin_package_version --bump patch foo',
                          'svn commit -m "0.1.2" ',
                          'svn cp -m "tagging 0.1.2" http://foo.bar/project1/trunk http://foo.bar/project1/tags/0.1.2'], commands)

        commands = get_commands('foo', {}, 'svn', '0.1.2', svn_url='http://foo.bar/project1/branches/branch_foo')
        self.assertEqual(['bin/catkin_package_version --bump patch foo',
                          'svn commit -m "0.1.2" ',
                          'svn cp -m "tagging 0.1.2" http://foo.bar/project1/branches/branch_foo http://foo.bar/project1/tags/0.1.2'], commands)

        commands = get_commands('foo', {}, 'svn', '0.1.2', svn_url='http://foo.bar/project1/tags/tag_foo')
        self.assertEqual(['bin/catkin_package_version --bump patch foo',
                          'svn commit -m "0.1.2" ',
                          'svn cp -m "tagging 0.1.2" http://foo.bar/project1/tags/tag_foo http://foo.bar/project1/tags/0.1.2'], commands)
