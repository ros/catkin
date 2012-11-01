import os
import stat
import unittest
import tempfile
import shutil
from mock import Mock

try:
    import catkin.environment_cache
    from catkin.environment_cache import _append_header, _append_footer, _set_variable, _append_comment, _is_not_windows, generate_static_environment_script, generate_environment_script
except ImportError as impe:
    raise ImportError(
        'Please adjust your pythonpath before running this test: %s' % str(impe))

class PlattformTest(unittest.TestCase):

    def setUp(self):
        self.platform_backup = catkin.environment_cache.platform
        self.winplatform = Mock()
        self.winplatform.system.return_value = 'Windows'
        linuxplatform = Mock()
        linuxplatform.system.return_value = 'Linux'
        catkin.environment_cache.platform = linuxplatform

    def tearDown(self):
        catkin.environment_cache.platform = self.platform_backup

    def test_is_not_windows(self):
        self.assertTrue(_is_not_windows())
        catkin.environment_cache.platform = self.winplatform
        self.assertFalse(_is_not_windows())

    def test_appends(self):
        code = []
        _append_header(code)
        self.assertEqual(['#!/usr/bin/env sh',
                          '# generated from catkin/python/catkin/environment_cache.py', ''], code)
        code = []
        _append_comment(code, 'foo')
        self.assertEqual(['# foo'], code)
        code = []
        _append_footer(code)
        self.assertEqual(['', 'exec "$@"'], code)
        code = []
        _set_variable(code, 'foo', 'bar')
        self.assertEqual(['export foo="bar"'], code)

    def test_appends_windows(self):
        catkin.environment_cache.platform = self.winplatform
        code = []
        _append_header(code)
        self.assertEqual(['@echo off',
                          'REM generated from catkin/python/catkin/environment_cache.py',
                          ''], code)
        code = []
        _append_comment(code, 'foo')
        self.assertEqual(['REM foo'], code)
        code = []
        _append_footer(code)
        self.assertEqual(['', '%*'], code)
        code = []
        _set_variable(code, 'foo', 'bar')
        self.assertEqual(['set foo="bar"'], code)

    def test_generate_environment_script(self):
        try:
            fake_environ = os.environ.copy()
            fake_environ['FOO'] = '/bar'
            fake_environ['TRICK'] = '/lib'
            catkin.environment_cache.os.environ = fake_environ
            rootdir = tempfile.mkdtemp()
            env_file = os.path.join(rootdir, 'env.sh')
            with open(env_file, 'ab') as fhand:
                fhand.write('''\
#! /usr/bin/env sh
export FOO=/foo:/bar
export TRICK=/usr/lib
export BAR=/bar
exec "$@"''')
            mode = os.stat(env_file).st_mode
            os.chmod(env_file, mode | stat.S_IXUSR)
            result = generate_environment_script(env_file)
            self.assertTrue('export FOO="/foo:$FOO"' in result, result)
            self.assertTrue('export TRICK="/usr/lib"' in result, result)
            self.assertTrue('export BAR="/bar"' in result, result)
            self.assertEqual('#!/usr/bin/env sh', result[0])
            self.assertEqual('exec "$@"', result[-1])
        finally:
            catkin.environment_cache.os.environ = os.environ
            shutil.rmtree(rootdir)

    def test_generate_static_environment_script_empty(self):
        try:
            fake_environ = os.environ.copy()
            fake_environ['CMAKE_PREFIX_PATH'] = ''
            fake_environ['CPATH'] = ''
            fake_environ['LD_LIBRARY_PATH'] = ''
            fake_environ['PATH'] = ''
            fake_environ['PKG_CONFIG_PATH'] = ''
            fake_environ['PYTHONPATH'] = ''
            catkin.environment_cache.os.environ = fake_environ
            result = generate_static_environment_script('/foo/build', ['/bar/install'], '/baz/pylib')
            self.assertTrue('export CMAKE_PREFIX_PATH="/bar/install:/foo/build"' in result, result)
            self.assertTrue('export CPATH="/foo/build/include"' in result, result)
            self.assertTrue('export LD_LIBRARY_PATH="/foo/build/lib"' in result, result)
            self.assertTrue('export PATH="/foo/build/bin"' in result, result)
            self.assertTrue('export PKG_CONFIG_PATH="/foo/build/lib/pkgconfig"' in result, result)
            self.assertTrue('export PYTHONPATH="/baz/pylib"' in result, result)
            self.assertEqual('#!/usr/bin/env sh', result[0])
            self.assertEqual('exec "$@"', result[-1])
            self.assertEqual(15, len(result))
        finally:
            catkin.environment_cache.os.environ = os.environ

    def test_generate_static_environment_script_foo(self):
        try:
            fake_environ = os.environ.copy()
            fake_environ['CMAKE_PREFIX_PATH'] = 'foo_cpp'
            fake_environ['CPATH'] = 'foo_cpath'
            fake_environ['LD_LIBRARY_PATH'] = 'foo_ld'
            fake_environ['PATH'] = 'foo_path'
            fake_environ['PKG_CONFIG_PATH'] = 'foo_pkgpath'
            fake_environ['PYTHONPATH'] = 'foo_pp'
            catkin.environment_cache.os.environ = fake_environ
            result = generate_static_environment_script('/foo/build', ['/bar/install'], '/baz/pylib')
            self.assertTrue('export CMAKE_PREFIX_PATH="/bar/install:/foo/build:foo_cpp"' in result, result)
            self.assertTrue('export CPATH="/foo/build/include:foo_cpath"' in result, result)
            self.assertTrue('export LD_LIBRARY_PATH="/foo/build/lib:foo_ld"' in result, result)
            self.assertTrue('export PATH="/foo/build/bin:foo_path"' in result, result)
            self.assertTrue('export PKG_CONFIG_PATH="/foo/build/lib/pkgconfig:foo_pkgpath"' in result, result)
            self.assertTrue('export PYTHONPATH="/baz/pylib:foo_pp"' in result, result)
            self.assertEqual('#!/usr/bin/env sh', result[0])
            self.assertEqual('exec "$@"', result[-1])
            self.assertEqual(15, len(result))
        finally:
            catkin.environment_cache.os.environ = os.environ
