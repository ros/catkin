# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Library to find packages in the filesystem.
"""

import os
from .package import parse_package


def find_package_paths(basepath):
    """
    Crawls the filesystem to find package.xml files.

    When a subfolder contains a file ``CATKIN_NO_SUBDIRS`` its
    subdirectories are ignored.

    :param basepath: The path to search in, ``str``
    :returns: A list of relative paths containing package.xml files
    ``list``
    """
    paths = []
    for dirpath, dirnames, filenames in os.walk(basepath, followlinks=True):
        if 'package.xml' in filenames:
            basename = os.path.basename(dirpath)
            if basename not in paths:
                paths.append(os.path.relpath(dirpath, basepath))
            del dirnames[:]
            continue
        elif 'CATKIN_NO_SUBDIRS' in filenames:
            del dirnames[:]
            continue
        for dirname in dirnames:
            if dirname.startswith('.'):
                dirnames.remove(dirname)
    return paths


def find_packages(basepath):
    """
    Crawls the filesystem to find package.xml files and parses them.

    :param basepath: The path to search in, ``str``
    :returns: A dict mapping relative paths to ``Package`` objects
    ``dict``
    :raises: :exc:RuntimeError` If multiple packages have the same
    name
    """
    packages = {}
    paths = find_package_paths(basepath)
    for path in paths:
        package = parse_package(os.path.join(basepath, path))
        if package.name in packages:
            raise RuntimeError('Two packages found with the same name "%s":\n- %s\n- %s' % (package.name, path.filename, packages[package.name].filename))
        packages[path] = package
    return packages


def verify_equal_package_versions(packages):
    """
    Verifies that all packages have the same version number.

    :param packages: The list of ``Package`` objects, ``list``
    :returns: The version number
    :raises: :exc:RuntimeError` If the version is not equal in all
    packages
    """
    version = None
    for package in packages:
        if version is None:
            version = package.version
        elif package.version != version:
            raise RuntimeError('Two packages have different version numbers (%s != %s):\n- %s' % (package.version, version, package.filename))
    return version
