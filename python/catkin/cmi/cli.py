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

import argparse

from catkin.cmi.context import Context
from catkin.cmi.build import build_isolated_workspace


def parse_args(args):
    parser = argparse.ArgumentParser(
        description=
        'Builds each catkin (and non-catkin) package from a given workspace in isolation, '
        'but still in topological order. Building of packages will be parallelized according to -p <# of jobs>.'
        'Make job flags (-j/-l) are extracted and passed to each make invocation.'
    )
    add = parser.add_argument
    add('--parallel-jobs', '--parallel', '-p', default=None,
        help='Maximum number of packages which could be built in parallel (default is cpu count)')
    add('--workspace', '-w', default=None,
        help='The base path of the workspace (default ".")')
    add('--source', '--source-space', default=None,
        help='The path to the source space (default "src")')
    add('--build', '--build-space', default=None,
        help='The path to the build space (default "build")')
    add('--devel', '--devel-space', default=None,
        help='Sets the target devel space (default "devel")')
    add('--merge-devel', action='store_true', default=False,
        help='Build each catkin package into a common devel space.')
    add('--install-space', dest='install_space', default=None,
        help='Sets the target install space (default "install")')
    add('--install', action='store_true', default=False,
        help='Causes each catkin package to be installed.')
    add('--isolate-install', action='store_true', default=False,
        help='Install each catkin package into a separate install space.')
    add('--force-cmake', action='store_true', default=False,
        help='Runs cmake explicitly for each catkin package.')
    add('--force-color', action='store_true', default=False,
        help='Forces cmi to ouput in color, even when the terminal does not appear to support it.')
    # pkg = parser.add_mutually_exclusive_group(required=False)
    # pkg.add_argument('--pkg', nargs='+', metavar='PKGNAME', dest='packages',
    #                  help='Invoke "make" on specific packages (only after catkin_make_isolated has been invoked before with the same install flag)')
    # pkg.add_argument('--from-pkg', metavar='PKGNAME', dest='from_package',
    #                  help='Restart catkin_make_isolated at the given package continuing from there (do not change CMake arguments, add/move/remove packages or toggle the install flag when using this option since this may result in an inconsistent workspace state).')
    # add('--only-pkg-with-deps', nargs='+', help='Only consider the specific packages and their recursive dependencies and ignore all other packages in the workspace (only works together with --merge or --install)')
    add('-v', '--verbose', action='store_true', default=False,
        help='Outputs output of each command in real-time, even if it interleaves with other output.')
    add('--cmake-args', dest='cmake_args', nargs='*', type=str,
        help='Arbitrary arguments which are passes to CMake. '
             'It must be passed after other arguments since it collects all following options.')
    add('--make-args', dest='make_args', nargs='*', type=str,
        help='Arbitrary arguments which are passes to make.'
             'It must be passed after other arguments since it collects all following options.')
    add('--catkin-make-args', dest='catkin_make_args', nargs='*', type=str,
        help='Arbitrary arguments which are passes to make but only for catkin packages.'
             'It must be passed after other arguments since it collects all following options.')
    opts = parser.parse_args(args)
    # if opts.only_pkg_with_deps and not opts.merge and not opts.install:
        # parser.error("The '--only-pkg-with-deps' option can only be used together with '--merge' or '--install'")
    return opts


def main(sysargs=None):
    opts = parse_args(sysargs)

    context = Context(
        workspace=opts.workspace,
        source_space=opts.source,
        build_space=opts.build,
        devel_space=opts.devel,
        merge_devel=opts.merge_devel,
        install_space=opts.install_space,
        install=opts.install,
        isolate_install=opts.isolate_install,
        cmake_args=opts.cmake_args,
        make_args=opts.make_args,
        catkin_make_args=opts.catkin_make_args
    )

    build_isolated_workspace(context, opts.parallel_jobs, opts.force_cmake, opts.force_color, opts.verbose)
