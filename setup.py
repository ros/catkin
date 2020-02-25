from catkin_pkg.python_setup import generate_distutils_setup

from setuptools import setup

import os

catkin_scripts = [
    'bin/catkin_find',
    'bin/catkin_init_workspace',
    'bin/catkin_make',
    'bin/catkin_make_isolated',
    'bin/catkin_test_results',
    'bin/catkin_topological_order',
]

if os.name == 'nt':
    import sys
    import shutil
    if sys.version_info.major == 2:
        # On Windows\Python2, Python scripts making use of multiprocessing
        # have to meet several requirements to work.
        # (catkin_pkg makes use of it to do parallel package parser.)
        #
        # One is that the scripts have to be ended with .py extension;
        # Otherwise, it will manifest as ImportError.
        if not os.path.isdir('build/bin'):
            os.makedirs('build/bin')
        for script in catkin_scripts:
            shutil.copyfile(script, 'build/%s.py' % script)
        catkin_scripts = ['build/%s.py' % script for script in catkin_scripts]

d = generate_distutils_setup(
    packages=['catkin'],
    package_dir={'': 'python'},
    scripts=catkin_scripts,
)

setup(**d)
