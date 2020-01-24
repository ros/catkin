from catkin_pkg.python_setup import generate_distutils_setup

from setuptools import setup

d = generate_distutils_setup(
    packages=['catkin'],
    package_dir={'': 'python'},
    scripts=[
        'bin/catkin_find',
        'bin/catkin_init_workspace',
        'bin/catkin_make',
        'bin/catkin_make_isolated',
        'bin/catkin_test_results',
        'bin/catkin_topological_order',
    ],
)

setup(**d)
