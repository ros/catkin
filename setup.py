#!/usr/bin/env python

from distutils.core import setup

from catkin.package import parse_package_for_distutils

d = parse_package_for_distutils()
d['packages'] = ['catkin']
d['package_dir'] = {'catkin': 'python/catkin'}
setup(**d)
