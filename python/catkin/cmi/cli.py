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
import re
import sys
import time

from catkin.cmi.color import clr
from catkin.cmi.color import set_color

from catkin.cmi.common import extract_cmake_and_make_and_catkin_make_arguments
from catkin.cmi.common import extract_jobs_flags
from catkin.cmi.common import format_time_delta
from catkin.cmi.common import get_build_type
from catkin.cmi.common import is_tty
from catkin.cmi.common import log

from catkin.cmi.context import Context

from catkin.cmi.build import build_isolated_workspace
from catkin.cmi.build import determine_packages_to_be_built
from catkin.cmi.build import topological_order_packages


def parse_args(args):
    # CMake/make pass-through flags collect dashed options. They require special
    # handling or argparse will complain about unrecognized options.
    args = sys.argv[1:] if args is None else args
    extract_make_args = extract_cmake_and_make_and_catkin_make_arguments
    args, cmake_args, make_args, catkin_make_args = extract_make_args(args)
    # Extract make jobs flags.
    jobs_flags = extract_jobs_flags(' '.join(args))
    if jobs_flags:
        args = re.sub(jobs_flags, '', ' '.join(args)).split()
        jobs_flags = jobs_flags.split()

    parser = argparse.ArgumentParser(
        description=
        'Builds each catkin (and non-catkin) package from a given workspace in isolation, '
        'but still in topological order. Building of packages will be parallelized according to -p <# of jobs>. '
        'Make job flags (-j/-l) are extracted and passed to each make invocation.'
    )
    add = parser.add_argument
    # What packages to build
    add('packages', nargs='*',
        help='Workspace packages to build, package dependencies are built as well unless --no-deps is used. '
             'If no packages are given, then all the packages are built.')
    add('--no-deps', action='store_true', default=False,
        help='Only build specified packages, not their dependencies.')
    add('--start-with', metavar='PKGNAME',
        help='Start building with this package, skipping any before it.')
    # Context options
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
    add('--space-suffix',
        help='suffix for build, devel, and install space if they are not otherwise explicitly set')
    # Build options
    add('--parallel-jobs', '--parallel', '-p', default=None,
        help='Maximum number of packages which could be built in parallel (default is cpu count)')
    add('--force-cmake', action='store_true', default=False,
        help='Runs cmake explicitly for each catkin package.')
    add('--lock-install', action='store_true', default=False,
        help='Prevents multiple jobs from simultaneously running install targets')
    add('--cmake-args', dest='cmake_args', nargs='*', type=str,
        help='Arbitrary arguments which are passes to CMake. '
             'It must be passed after other arguments since it collects all following options.')
    add('--make-args', dest='make_args', nargs='*', type=str,
        help='Arbitrary arguments which are passes to make.'
             'It must be passed after other arguments since it collects all following options.')
    add('--catkin-make-args', dest='catkin_make_args', nargs='*', type=str,
        help='Arbitrary arguments which are passes to make but only for catkin packages.'
             'It must be passed after other arguments since it collects all following options.')
    # Behavior
    add('--force-color', action='store_true', default=False,
        help='Forces cmi to ouput in color, even when the terminal does not appear to support it.')
    add('--quiet', '-q', action='store_true', default=False,
        help='Supresses output from commands unless there is an error.')
    add('--interleave-output', '-i', action='store_true', default=False,
        help='Prevents ordering of command output when multiple commands are running at the same time.')
    add('--no-status', action='store_true', default=False,
        help='Suppresses status line, useful in situations where carriage return is not properly supported.')
    # Commands
    add('--list-only', '--list', action='store_true', default=False,
        help='List packages in topological order, then exit.')

    opts = parser.parse_args(args)
    opts.cmake_args = cmake_args
    opts.make_args = make_args + (jobs_flags or [])
    opts.catkin_make_args = catkin_make_args

    if opts.no_deps and not opts.packages:
        sys.exit("With --no-deps, you must specify packages to build.")
    return opts


def list_only(context, packages, no_deps, start_with):
    # Print Summary
    log(context.summary())
    # Find list of packages in the workspace
    packages_to_be_built, packages_to_be_built_deps = determine_packages_to_be_built(packages, context)
    if not no_deps:
        # Extend packages to be built to include their deps
        packages_to_be_built.extend(packages_to_be_built_deps)
        # Also resort
        packages_to_be_built = topological_order_packages(dict(packages_to_be_built))
    # Print packages
    log("Packages to be built:")
    max_name_len = str(max([len(pkg.name) for pth, pkg in packages_to_be_built]))
    prefix = clr('@{pf}' + ('------ ' if start_with else '- ') + '@|')
    for pkg_path, pkg in packages_to_be_built:
        build_type = get_build_type(pkg)
        if build_type == 'catkin' and 'metapackage' in [e.tagname for e in pkg.exports]:
            build_type = 'metapackage'
        if start_with and pkg.name == start_with:
            start_with = None
        log(clr("{prefix}@{cf}{name:<" + max_name_len + "}@| (@{yf}{build_type}@|)")
            .format(prefix=clr('@!@{kf}(skip)@| ') if start_with else prefix, name=pkg.name, build_type=build_type))
    log("Total packages: " + str(len(packages_to_be_built)))


def main(sysargs=None):
    opts = parse_args(sysargs)

    if not opts.force_color and not is_tty(sys.stdout):
        set_color(False)

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
        catkin_make_args=opts.catkin_make_args,
        space_suffix=opts.space_suffix
    )

    if opts.list_only:
        list_only(context, opts.packages, opts.no_deps, opts.start_with)
        return

    start = time.time()
    try:
        return build_isolated_workspace(
            context,
            packages=opts.packages,
            start_with=opts.start_with,
            no_deps=opts.no_deps,
            jobs=opts.parallel_jobs,
            force_cmake=opts.force_cmake,
            force_color=opts.force_color,
            quiet=opts.quiet,
            interleave_output=opts.interleave_output,
            no_status=opts.no_status,
            lock_install=opts.lock_install
        )
    except Exception:
        raise
    finally:
        log("[cmi] Runtime: {0}".format(format_time_delta(time.time() - start)))
