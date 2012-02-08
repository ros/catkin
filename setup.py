#!/usr/bin/env python

from distutils.core import setup

setup(name='catkin',
      version= "0.0.0",
      packages=['catkin', 'catkin.sphinx'],
      package_dir = {'catkin':'python/catkin',
                     'catkin.sphinx': 'python/catkin/sphinx'},
      scripts = [],
      author = "Troy Straszheim, Morten Kjaergaard", 
      author_email = "straszheim@willowgarage.com",
      url = "http://www.ros.org",
      download_url = "http://github.com/straszheim/catkin/", 
      keywords = ["ROS"],
      classifiers = [
        "Programming Language :: Python", 
        "License :: OSI Approved :: BSD License" ],
      description = "ROS msg/srv generation", 
      long_description = """\
Build system stuff
""",
      license = "BSD"
      )

