import os
import unittest

import imp
imp.load_source('catkin_make_isolated',
                os.path.join(os.path.dirname(__file__),
                             '..', '..', 'bin', 'catkin_make_isolated'))

from catkin_make_isolated import extract_cmake_args
from catkin_make_isolated import parse_args
from catkin_make_isolated import handle_cmake_args


class CatkinMakeIsolatedTests(unittest.TestCase):

    def test_extract_cmake_args(self):
        args = []
        cmake_args, args = extract_cmake_args(args)
        assert cmake_args == []
        assert args == []

        args = ['-DCMAKE_INSTALL_PREFIX=install']
        cmake_args, args = extract_cmake_args(args)
        assert cmake_args == ['-DCMAKE_INSTALL_PREFIX=install']
        assert args == []

        args = ['-DCMAKE_INSTALL_PREFIX=install', '--install']
        cmake_args, args = extract_cmake_args(args)
        assert cmake_args == ['-DCMAKE_INSTALL_PREFIX=install']
        assert args == ['--install']

        args = [
            '-DCMAKE_INSTALL_PREFIX=install', '--install', '--install-space',
            'install_isolated'
        ]
        cmake_args, args = extract_cmake_args(args)
        assert cmake_args == ['-DCMAKE_INSTALL_PREFIX=install']
        assert args == ['--install', '--install-space', 'install_isolated']

        args = ['-DCATKIN_DEVEL_PREFIX=devel']
        cmake_args, args = extract_cmake_args(args)
        assert cmake_args == ['-DCATKIN_DEVEL_PREFIX=devel']
        assert args == []

        args = ['-DCATKIN_DEVEL_PREFIX=devel']
        cmake_args, args = extract_cmake_args(args)
        assert cmake_args == ['-DCATKIN_DEVEL_PREFIX=devel']
        assert args == []

        args = [
            '-DCATKIN_DEVEL_PREFIX=devel', '--devel-space',
            'devel_isolated'
        ]
        cmake_args, args = extract_cmake_args(args)
        assert cmake_args == ['-DCATKIN_DEVEL_PREFIX=devel']
        assert args == ['--devel-space', 'devel_isolated']

    def test_handle_cmake_args(self):
        args = ['-DCMAKE_INSTALL_PREFIX=install', '--install']
        cmake_args, args = extract_cmake_args(args)
        assert cmake_args == ['-DCMAKE_INSTALL_PREFIX=install'], cmake_args
        opts = parse_args(args)
        cmake_args, opts = handle_cmake_args(cmake_args, opts)
        assert cmake_args == [], cmake_args
        assert opts.install == True
        assert opts.install_space == 'install'

        args = [
            '-DCMAKE_INSTALL_PREFIX=install', '--install', '--install-space',
            'install_isolated'
        ]
        cmake_args, args = extract_cmake_args(args)
        assert cmake_args == ['-DCMAKE_INSTALL_PREFIX=install'], cmake_args
        opts = parse_args(args)
        cmake_args, opts = handle_cmake_args(cmake_args, opts)
        assert cmake_args == [], cmake_args
        assert opts.install == True
        assert opts.install_space == 'install_isolated'

        args = ['-DCATKIN_DEVEL_PREFIX=devel']
        cmake_args, args = extract_cmake_args(args)
        assert cmake_args == ['-DCATKIN_DEVEL_PREFIX=devel'], cmake_args
        opts = parse_args(args)
        cmake_args, opts = handle_cmake_args(cmake_args, opts)
        assert cmake_args == [], cmake_args
        assert opts.devel == 'devel'

        args = [
            '-DCATKIN_DEVEL_PREFIX=devel', '--devel-space',
            'devel_isolated'
        ]
        cmake_args, args = extract_cmake_args(args)
        assert cmake_args == ['-DCATKIN_DEVEL_PREFIX=devel'], cmake_args
        opts = parse_args(args)
        cmake_args, opts = handle_cmake_args(cmake_args, opts)
        assert cmake_args == [], cmake_args
        assert opts.devel == 'devel_isolated'
