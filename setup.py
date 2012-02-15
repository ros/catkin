#!/usr/bin/env python

from distutils.core import setup

import re
search = re.search(r'Version\:\s*(\d+\.\d+\.\d+)',open('stack.yaml').read())
if not search:
    print >>sys.stderr, 'You must have a Version field in your stack.yaml'
    sys.exit(-1)
__version__ = search.groups()[0]

setup(name='catkin',
      version= __version__,
      packages=['catkin'],
      package_dir = {'catkin':'python/catkin',
                     },
      scripts = [],
      author = "Troy Straszheim, Morten Kjaergaard", 
      author_email = "straszheim@willowgarage.com",
      url = "http://www.ros.org",
      download_url = "http://github.com/straszheim/catkin/", 
      keywords = ["ROS"],
      classifiers = [
        "Programming Language :: Python", 
        "License :: OSI Approved :: BSD License" ],
      description = "Catkin cmake library", 
      long_description = """\
Build system stuff
""",
      license = "BSD"
      )

