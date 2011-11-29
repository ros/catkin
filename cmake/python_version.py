from __future__ import print_function
from sys import version_info as v
# don't use version.major, version.minor, this is pys >= 2.7
print("%u.%u" % (v[0], v[1]))
 
