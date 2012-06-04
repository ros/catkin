#!/usr/bin/env python

from distutils.core import setup
import sys
from xml.etree.ElementTree import ElementTree

try:
    root = ElementTree(None, 'stack.xml')
    version = root.findtext('version')
except Exception, e:
    print >> sys.stderr, 'Could not extract version from your stack.xml:\n%s' % e
    sys.exit(-1)

setup(name='catkin',
      version = version,
      packages = ['catkin'],
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

