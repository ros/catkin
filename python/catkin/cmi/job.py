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

from __future__ import print_function

import os
import stat

from catkin.cmi.common import create_build_space
from catkin.cmi.common import get_cached_recursive_build_depends_in_workspace
from catkin.cmi.common import get_python_install_dir
from catkin.cmi.common import handle_make_arguments
from catkin.cmi.common import which

CMAKE_EXEC = which('cmake')
if CMAKE_EXEC is None:
    raise RuntimeError("Executable 'cmake' could not be found in PATH.")
MAKE_EXEC = which('make')
if MAKE_EXEC is None:
    raise RuntimeError("Executable 'make' could not be found in PATH.")


class Command(object):
    """Single command which is part of a job"""
    lock_install_space = False

    def __init__(self, cmd, location):
        self.cmd = cmd
        self.location = location


class InstallCommand(Command):
    """Command which touches the install space"""
    lock_install_space = True

    def __init__(self, cmd, location):
        super(InstallCommand, self).__init__(cmd, location)


class Job(object):
    """Encapsulates a job which builds a package"""
    def __init__(self, package, package_path, context, force_cmake):
        self.package = package
        self.package_path = package_path
        self.context = context
        self.force_cmake = force_cmake
        self.commands = []
        self.__command_index = 0

    def get_commands(self):
        raise NotImplementedError('get_commands')

    def __iter__(self):
        return self

    def __next__(self):
        return self.next()

    def next(self):
        if self.__command_index >= len(self.commands):
            raise StopIteration()
        self.__command_index += 1
        return self.commands[self.__command_index - 1]


def create_env_file(package, context):
    sources = []
    # If installing to isolated folders or not installing, but devel spaces are not merged
    if (context.install and context.isolate_install) or (not context.install and not context.merge_devel):
        # Source each package's install or devel space
        space = context.install_space if context.install else context.devel_space
        # Get the recursive dependcies
        depends = get_cached_recursive_build_depends_in_workspace(package, context.workspace_packages)
        # For each dep add a line to source its setup file
        source_snippet = ". {source_path} --extend\n"
        for dep_pth, dep in depends:
            source_path = os.path.join(space, dep.name, 'setup.sh')
            sources.append(source_snippet.format(source_path=source_path))
    else:
        # Just source common install or devel space
        source_snippet = ". {source_path}\n"
        source_path = os.path.join(context.install_space if context.install else context.devel_space, 'setup.sh')
        sources = [source_snippet.format(source_path=source_path)] if os.path.exists(source_path) else []
    # Build the env_file
    env_file_path = os.path.abspath(os.path.join(context.build_space, package.name, 'cmi_env.sh'))
    env_file = """\
#!/usr/bin/env sh
# generated from within catkin/python/catkin/cmi/job.py

if [ $# -eq 0 ] ; then
  /bin/echo "Usage: cmi_env.sh COMMANDS"
  /bin/echo "Calling cmi_env.sh without arguments is not supported anymore."
  /bin/echo "Instead spawn a subshell and source a setup file manually."
  exit 1
fi

# source setup.sh with --extend argument for each direct build depend in the workspace
{sources}
exec "$@"
""".format(sources=''.join(sources))
    with open(env_file_path, 'w') as f:
        f.write(env_file)
    # Make this file executable
    os.chmod(env_file_path, stat.S_IXUSR | stat.S_IWUSR | stat.S_IRUSR)
    return env_file_path


# TODO: Move various Job types out to another file

class CMakeJob(Job):
    """Job class for building plain cmake packages"""
    def __init__(self, package, package_path, context, force_cmake):
        Job.__init__(self, package, package_path, context, force_cmake)
        self.commands = self.get_commands()

    def get_commands(self):
        commands = []
        # Setup build variables
        pkg_dir = os.path.join(self.context.source_space, self.package_path)
        build_space = create_build_space(self.context.build_space, self.package.name)
        if self.context.merge_devel:
            devel_space = self.context.devel_space
        else:
            devel_space = os.path.join(self.context.devel_space, self.package.name)
        if self.context.isolate_install:
            install_space = os.path.join(self.context.install_space, self.package.name)
        else:
            install_space = self.context.install_space
        install_target = install_space if self.context.install else devel_space
        # Create an environment file
        env_cmd = create_env_file(self.package, self.context)
        # CMake command
        makefile_path = os.path.join(build_space, 'Makefile')
        if not os.path.isfile(makefile_path) or self.force_cmake:
            commands.append(Command(
                [
                    env_cmd,
                    CMAKE_EXEC,
                    pkg_dir,
                    '-DCMAKE_INSTALL_PREFIX=' + install_target
                ] + self.context.cmake_args,
                build_space
            ))
            commands[-1].cmd.extend(self.context.cmake_args)
        else:
            commands.append(Command([env_cmd, MAKE_EXEC, 'cmake_check_build_system'], build_space))
        # Make command
        commands.append(Command(
            [env_cmd, MAKE_EXEC] + handle_make_arguments(self.context.make_args),
            build_space
        ))
        # Make install command (always run on plain cmake)
        commands.append(InstallCommand([env_cmd, MAKE_EXEC, 'install'], build_space))
        # Determine the location of where the setup.sh file should be created
        if self.context.install:
            setup_file_path = os.path.join(install_space, 'setup.sh')
            if not self.context.isolate_install and os.path.exists(setup_file_path):
                return commands
        else:  # Create it in the devel space
            setup_file_path = os.path.join(devel_space, 'setup.sh')
            if self.context.merge_devel and os.path.exists(setup_file_path):
                # Do not replace existing setup.sh if devel space is merged
                return commands
        # Create the setup file other packages will source when depending on this package
        subs = {}
        subs['cmake_prefix_path'] = install_target + ":"
        subs['ld_path'] = os.path.join(install_target, 'lib') + ":"
        pythonpath = os.path.join(install_target, get_python_install_dir())
        subs['pythonpath'] = pythonpath + ':'
        subs['pkgcfg_path'] = os.path.join(install_target, 'lib', 'pkgconfig')
        subs['pkgcfg_path'] += ":"
        subs['path'] = os.path.join(install_target, 'bin') + ":"
        setup_file_directory = os.path.dirname(setup_file_path)
        if not os.path.exists(setup_file_directory):
            os.makedirs(setup_file_directory)
        with open(setup_file_path, 'w') as file_handle:
            file_handle.write("""\
#!/usr/bin/env sh
# generated from catkin.builder module

# remember type of shell if not already set
if [ -z "$CATKIN_SHELL" ]; then
  CATKIN_SHELL=sh
fi

# detect if running on Darwin platform
_UNAME=`uname -s`
IS_DARWIN=0
if [ "$_UNAME" = "Darwin" ]; then
  IS_DARWIN=1
fi

# Prepend to the environment
export CMAKE_PREFIX_PATH="{cmake_prefix_path}$CMAKE_PREFIX_PATH"
if [ $IS_DARWIN -eq 0 ]; then
  export LD_LIBRARY_PATH="{ld_path}$LD_LIBRARY_PATH"
else
  export DYLD_LIBRARY_PATH="{ld_path}$DYLD_LIBRARY_PATH"
fi
export PATH="{path}$PATH"
export PKG_CONFIG_PATH="{pkgcfg_path}$PKG_CONFIG_PATH"
export PYTHONPATH="{pythonpath}$PYTHONPATH"
""".format(**subs))
        return commands


class CatkinJob(Job):
    """Job class for building catkin packages"""
    def __init__(self, package, package_path, context, force_cmake):
        Job.__init__(self, package, package_path, context, force_cmake)
        self.commands = self.get_commands()

    def get_commands(self):
        commands = []
        # Setup build variables
        pkg_dir = os.path.join(self.context.source_space, self.package_path)
        build_space = create_build_space(self.context.build_space, self.package.name)
        if self.context.merge_devel:
            devel_space = self.context.devel_space
        else:
            devel_space = os.path.join(self.context.devel_space, self.package.name)
        if self.context.isolate_install:
            install_space = os.path.join(self.context.install_space, self.package.name)
        else:
            install_space = self.context.install_space
        # Create an environment file
        env_cmd = create_env_file(self.package, self.context)
        # CMake command
        makefile_path = os.path.join(build_space, 'Makefile')
        if not os.path.isfile(makefile_path) or self.force_cmake:
            commands.append(Command(
                [
                    env_cmd,
                    CMAKE_EXEC,
                    pkg_dir,
                    '-DCATKIN_DEVEL_PREFIX=' + devel_space,
                    '-DCMAKE_INSTALL_PREFIX=' + install_space
                ] + self.context.cmake_args,
                build_space
            ))
        else:
            commands.append(Command([env_cmd, MAKE_EXEC, 'cmake_check_build_system'], build_space))
        # Make command
        commands.append(Command(
            [env_cmd, MAKE_EXEC] + handle_make_arguments(self.context.make_args),
            build_space
        ))
        # Make install command, if installing
        if self.context.install:
            commands.append(InstallCommand([env_cmd, MAKE_EXEC, 'install'], build_space))
        return commands
