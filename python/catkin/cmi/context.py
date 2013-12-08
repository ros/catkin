# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Open Source Robotics Foundation, Inc.
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
#  * Neither the name of Open Source Robotics Foundation, Inc. nor
#    the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior
#    written permission.
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

"""This module implements a class for representing a catkin workspace context"""

from __future__ import print_function

import os


# TODO: extend builtin prototype to handle locking
class Context(object):
    """Encapsulates a catkin workspace's settings which affect build results.

    This class will validate some of the settings on assignment using the
    filesystem, but it will never modify the filesystem. For instance, it will
    raise an exception if the source space does not exist, but it will not
    create a folder for the build space if it does not already exist.

    This context can be locked, so that changing the members is prevented.
    """
    def __init__(
        self,
        workspace=None,
        source_space=None,
        build_space=None,
        devel_space=None,
        install_space=None,
        merge_devel=False,
        install=False,
        isolate_install=False,
        cmake_args=None,
        make_args=None,
        catkin_make_args=None,
        space_suffix=None
    ):
        """Creates a new Context object, optionally initializing with parameters

        :param workspace: root of the workspace, defaults to '.'
        :type workspace: str
        :param source_space: location of source space, defaults to '<workspace>/src'
        :type source_space: str
        :param build_space: target location of build space, defaults to '<workspace>/build'
        :type build_space: str
        :param devel_space: target location of devel space, defaults to '<workspace>/devel'
        :type devel_space: str
        :param install_space: target location of install space, defaults to '<workspace>/install'
        :type install_space: str
        :param merge_devel: devel space will be shared for all packages if True, default is False
        :type merge_devel: bool
        :param install: packages will be installed by invoking ``make install``, defaults to False
        :type install: bool
        :param isolate_install: packages will be installed to separate folders if True, defaults to False
        :type isolate_install: bool
        :param cmake_args: extra cmake arguments to be passed to cmake for each package
        :type cmake_args: list
        :param make_args: extra make arguments to be passed to make for each package
        :type make_args: list
        :param catkin_make_args: extra make arguments to be passed to make for each catkin package
        :type catkin_make_args: list
        :param space_suffix: suffix for build, devel, and install spaces which are not explicitly set.
        :type space_suffix: str
        :raises: ValueError if workspace or source space does not exist
        """
        self.__locked = False
        ss = '' if space_suffix is None else space_suffix
        # Validation is done on assignment
        # Handle *space assignment and defaults
        self.workspace = '.' if workspace is None else workspace
        self.source_space = os.path.join(self.workspace, 'src') if source_space is None else source_space
        self.build_space = os.path.join(self.workspace, 'build' + ss) if build_space is None else build_space
        self.devel_space = os.path.join(self.workspace, 'devel' + ss) if devel_space is None else devel_space
        self.install_space = os.path.join(self.workspace, 'install' + ss) if install_space is None else install_space
        self.destdir = os.environ['DESTDIR'] if 'DESTDIR' in os.environ else None
        # Handle build options
        self.merge_devel = merge_devel
        self.install = install
        self.isolate_install = isolate_install
        # Handle additional cmake and make arguments
        self.cmake_args = cmake_args or []
        self.make_args = make_args or []
        self.catkin_make_args = catkin_make_args or []
        # List of packages in the workspace is set externally
        self.packages = []

    def summary(self):
        summary = [
            [
                "Workspace:                   {_Context__workspace}",
                "Buildspace:                  {_Context__build_space}",
                "Develspace:                  {_Context__devel_space}",
                "Installspace:                {_Context__install_space}",
                "DESTDIR:                     {_Context__destdir}"
            ],
            [
                "Merge Develspaces:           {_Context__merge_devel}",
                "Install Packages:            {_Context__install}",
                "Isolate Installs:            {_Context__isolate_install}"
            ],
            [
                "Additional CMake Args:       {cmake_args}",
                "Additional Make Args:        {make_args}",
                "Additional catkin Make Args: {catkin_make_args}"
            ]
        ]
        subs = {
            'cmake_args': ', '.join(self.__cmake_args or ['None']),
            'make_args': ', '.join(self.__make_args or ['None']),
            'catkin_make_args': ', '.join(self.__catkin_make_args or ['None'])
        }
        subs.update(**self.__dict__)
        max_length = 0
        groups = []
        for group in summary:
            for index, line in enumerate(group):
                group[index] = line.format(**subs)
                max_length = max(max_length, len(line))
            groups.append("\n".join(group))
        return ('-' * max_length) + "\n" + ("\n" + ('-' * max_length) + "\n").join(groups) + "\n" + ('-' * max_length)

    @property
    def workspace(self):
        return self.__workspace

    @workspace.setter
    def workspace(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        # Validate Workspace
        if not os.path.exists(value):
            raise ValueError("Workspace path '{0}' does not exist.".format(value))
        self.__workspace = os.path.abspath(value)

    @property
    def source_space(self):
        return self.__source_space

    @source_space.setter
    def source_space(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        # Check that the source space exists
        if not os.path.exists(value):
            raise ValueError("Could not find source space: {0}".format(value))
        self.__source_space = os.path.abspath(value)

    @property
    def build_space(self):
        return self.__build_space

    @build_space.setter
    def build_space(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        # TODO: check that build space was not run with a different context before
        self.__build_space = value

    @property
    def devel_space(self):
        return self.__devel_space

    @devel_space.setter
    def devel_space(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        # TODO: check that devel space was not run with a different context before
        self.__devel_space = value

    @property
    def install_space(self):
        return self.__install_space

    @install_space.setter
    def install_space(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        # TODO: check that install space was not run with a different context before
        self.__install_space = value

    @property
    def destdir(self):
        return self.__destdir

    @destdir.setter
    def destdir(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        self.__destdir = value

    @property
    def merge_devel(self):
        return self.__merge_devel

    @merge_devel.setter
    def merge_devel(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        self.__merge_devel = value

    @property
    def install(self):
        return self.__install

    @install.setter
    def install(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        self.__install = value

    @property
    def isolate_install(self):
        return self.__isolate_install

    @isolate_install.setter
    def isolate_install(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        self.__isolate_install = value

    @property
    def cmake_args(self):
        return self.__cmake_args

    @cmake_args.setter
    def cmake_args(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        self.__cmake_args = value

    @property
    def make_args(self):
        return self.__make_args

    @make_args.setter
    def make_args(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        self.__make_args = value

    @property
    def catkin_make_args(self):
        return self.__catkin_make_args

    @catkin_make_args.setter
    def catkin_make_args(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        self.__catkin_make_args = value

    @property
    def packages(self):
        return self.__packages

    @packages.setter
    def packages(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        self.__packages = value
