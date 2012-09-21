from __future__ import print_function
import os
import re
from xml.etree.ElementTree import ElementTree


def get_version(path):
    package_path = os.path.join(path, 'package.xml')
    if not os.path.exists(package_path):
        raise RuntimeError('Could not find package.xml in current folder.')

    try:
        root = ElementTree(None, package_path)
        return root.findtext('version')
    except Exception as e:
        raise RuntimeError('Could not extract version from package.xml:\n%s' % e)


def bump_version(version, bump='patch'):
    # split the version number
    new_version = re.search('(\d+)\.(\d+)\.(\d+)', version).groups()
    new_version = [int(x) for x in new_version]
    # find the desired index
    idx = dict(major=0, minor=1, patch=2)[bump]
    # increment the desired part
    new_version[idx] += 1
    # reset all parts behind the bumped part
    new_version = new_version[:idx + 1] + [0 for x in new_version[idx + 1:]]
    return '%d.%d.%d' % tuple(new_version)


def update_version(path, new_version):
    package_path = os.path.join(path, 'package.xml')
    with open(package_path, 'r') as f:
        package_str = f.read()
    # write back modified package.xml
    with open(package_path, 'w') as f:
        new_package_str, number_of_subs = re.subn('<version([^<>]*)>[^<>]*</version>', '<version\g<1>>%s</version>' % new_version, package_str, 1)
        if number_of_subs != 1:
            raise RuntimeError('Could not bump version number')
        f.write(new_package_str)
