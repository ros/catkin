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

import datetime
import multiprocessing
import os
import re
import sys

if os.name == 'nt':
    import catkin.cmi.run_windows as run
else:
    import catkin.cmi.run_unix as run

# Get platform specific run command
run_command = run.run_command


class FakeLock(object):
    """Fake lock used to mimic a Lock but without causing synchronization"""
    def acquire(self, blocking=False):
        return True

    def release(self):
        pass

    def __enter__(self):
        pass

    def __exit__(self, exc_type, exc_value, traceback):
        pass


def create_build_space(buildspace, package_name):
    package_build_dir = os.path.join(buildspace, package_name)
    if not os.path.exists(package_build_dir):
        os.makedirs(package_build_dir)
    return package_build_dir


def extract_cmake_and_make_arguments(args):
    args, cmake_args, make_args, _ = _extract_cmake_and_make_arguments(args, extract_catkin_make=False)
    return args, cmake_args, make_args


def extract_cmake_and_make_and_catkin_make_arguments(args):
    return _extract_cmake_and_make_arguments(args, extract_catkin_make=True)


def split_arguments(args, splitter_name, default=None):
    if splitter_name not in args:
        return args, default
    index = args.index(splitter_name)
    return args[0:index], args[index + 1:]


def _extract_cmake_and_make_arguments(args, extract_catkin_make):
    cmake_args = []
    make_args = []
    catkin_make_args = []

    arg_types = {
        '--cmake-args': cmake_args,
        '--make-args': make_args
    }
    if extract_catkin_make:
        arg_types['--catkin-make-args'] = catkin_make_args

    arg_indexes = {}
    for k in arg_types.keys():
        if k in args:
            arg_indexes[args.index(k)] = k

    for index in reversed(sorted(arg_indexes.keys())):
        arg_type = arg_indexes[index]
        args, specific_args = split_arguments(args, arg_type)
        arg_types[arg_type].extend(specific_args)

    # classify -D* and -G* arguments as cmake specific arguments
    implicit_cmake_args = [a for a in args if a.startswith('-D') or a.startswith('-G')]
    args = [a for a in args if a not in implicit_cmake_args]

    return args, implicit_cmake_args + cmake_args, make_args, catkin_make_args


def extract_jobs_flags(mflags):
    regex = r'(?:^|\s)(-?(?:j|l)(?:\s*[0-9]+|\s|$))' + \
            r'|' + \
            r'(?:^|\s)((?:--)?(?:jobs|load-average)(?:(?:=|\s+)[0-9]+|(?:\s|$)))'
    matches = re.findall(regex, mflags) or []
    matches = [m[0] or m[1] for m in matches]
    return ' '.join([m.strip() for m in matches]) if matches else None


def format_time_delta(delta):
    hours, minutes, seconds = str(datetime.timedelta(seconds=delta)).split(':')
    msg = "" if int(hours) == 0 else (hours + ":")
    msg += "" if int(minutes) == 0 else (minutes + ":")
    msg += "{0:2.1f}".format(float(seconds))
    return msg


def handle_make_arguments(input_make_args, force_single_threaded_when_running_tests=False):
    make_args = list(input_make_args)
    # make_args = ['-j1']

    if force_single_threaded_when_running_tests:
        # force single threaded execution when running test since rostest does not support multiple parallel runs
        run_tests = [a for a in make_args if a.startswith('run_tests')]
        if run_tests:
            print('Forcing "-j1" for running unit tests.')
            make_args.append('-j1')

    # If no -j/--jobs/-l/--load-average flags are in make_args
    if not extract_jobs_flags(' '.join(make_args)):
        # If -j/--jobs/-l/--load-average are in MAKEFLAGS
        if 'MAKEFLAGS' in os.environ and extract_jobs_flags(os.environ['MAKEFLAGS']):
            # Do not extend make arguments, let MAKEFLAGS set things
            pass
        else:
            # Else extend the make_arguments to include some jobs flags
            # If ROS_PARALLEL_JOBS is set use those flags
            if 'ROS_PARALLEL_JOBS' in os.environ:
                # ROS_PARALLEL_JOBS is a set of make variables, not just a number
                ros_parallel_jobs = os.environ['ROS_PARALLEL_JOBS']
                make_args.extend(ros_parallel_jobs.split())
            else:
                # Else Use the number of CPU cores
                try:
                    jobs = multiprocessing.cpu_count()
                    make_args.append('-j{0}'.format(jobs))
                    make_args.append('-l{0}'.format(jobs))
                except NotImplementedError:
                    # If the number of cores cannot be determined, do not extend args
                    pass
    return make_args


def get_build_type(package):
    export_tags = [e.tagname for e in package.exports]
    if 'build_type' in export_tags:
        build_type_tag = [e.content for e in package.exports if e.tagname == 'build_type'][0]
    else:
        build_type_tag = 'catkin'
    return build_type_tag


def get_python_install_dir():
    # this function returns the same value as the CMake variable PYTHON_INSTALL_DIR from catkin/cmake/python.cmake
    python_install_dir = 'lib'
    if os.name != 'nt':
        python_version_xdoty = str(sys.version_info[0]) + '.' + str(sys.version_info[1])
        python_install_dir = os.path.join(python_install_dir, 'python' + python_version_xdoty)

    python_use_debian_layout = os.path.exists('/etc/debian_version')
    python_packages_dir = 'dist-packages' if python_use_debian_layout else 'site-packages'
    python_install_dir = os.path.join(python_install_dir, python_packages_dir)
    return python_install_dir

__recursive_depends_cache = {}


def get_cached_recursive_build_depends_in_workspace(package, workspace_packages):
    workspace_key = ','.join([pkg.name for pth, pkg in workspace_packages])
    if workspace_key not in __recursive_depends_cache:
        __recursive_depends_cache[workspace_key] = {}
    cache = __recursive_depends_cache[workspace_key]
    if package.name not in cache:
        cache[package.name] = get_recursive_build_depends_in_workspace(package, workspace_packages)
    __recursive_depends_cache[workspace_key] = cache
    return __recursive_depends_cache[workspace_key][package.name]


def get_recursive_build_depends_in_workspace(package, ordered_packages):
    workspace_packages_by_name = dict([(pkg.name, (pth, pkg)) for pth, pkg in ordered_packages])
    workspace_package_names = [pkg.name for pth, pkg in ordered_packages]
    recursive_depends = []
    depends = set([dep.name for dep in (package.build_depends + package.buildtool_depends)])
    checked_depends = set()
    while list(depends - checked_depends):
        # Get a dep
        dep = list(depends - checked_depends).pop()
        # Add the dep to the checked list
        checked_depends.add(dep)
        # If it is not in the workspace, continue
        if dep not in workspace_package_names:
            continue
        # Add the build, buildtool, and run depends of this dep to the list to be checked
        dep_pth, dep_pkg = workspace_packages_by_name[dep]
        dep_depends = dep_pkg.build_depends + dep_pkg.buildtool_depends + dep_pkg.run_depends
        depends.update(set([d.name for d in dep_depends]))
        # Add this package to the list of recursive dependencies for this package
        recursive_depends.append((dep_pth, dep_pkg))
    return recursive_depends


def log(*args, **kwargs):
    if 'end_with_escape' not in kwargs or kwargs['end_with_escape'] is True:
        args = list(args)
        args.append('\033[0m')
        if 'end_with_escape' in kwargs:
            del kwargs['end_with_escape']
    print(*args, **kwargs)


def terminal_width_windows():
    """Returns the estimated width of the terminal on Windows"""
    from ctypes import windll, create_string_buffer
    h = windll.kernel32.GetStdHandle(-12)
    csbi = create_string_buffer(22)
    res = windll.kernel32.GetConsoleScreenBufferInfo(h, csbi)

    #return default size if actual size can't be determined
    if not res:
        return 80, 25

    import struct
    (bufx, bufy, curx, cury, wattr, left, top, right, bottom, maxx, maxy)\
        = struct.unpack("hhhhHhhhhhh", csbi.raw)
    width = right - left + 1

    return width


def terminal_width_linux():
    """Returns the estimated width of the terminal on linux"""
    width = os.popen('tput cols', 'r').readline()

    return int(width)


def terminal_width():
    """Returns the estimated width of the terminal"""
    return terminal_width_windows() if os.name == 'nt' else terminal_width_linux()


def remove_ansi_escape(string):
    ansi_escape = re.compile(r'\x1b[^m]*m')
    return ansi_escape.sub('', string)


def wide_log(msg, **kwargs):
    width = terminal_width()
    if 'truncate' in kwargs:
        if kwargs['truncate'] and len(msg) >= width - 1:
            msg = msg[:width - 4] + '...'
        del kwargs['truncate']
    if len(msg) < width:
        log(msg + (' ' * (width - len(remove_ansi_escape(msg)) - 1)), **kwargs)
    else:
        log(msg, **kwargs)


def which(program):
    """Custom version of the ``which`` built-in shell command.

    Searches the pathes in the ``PATH`` environment variable for a given
    executable name. It returns the full path to the first instance of the
    executable found or None if it was not found.

    :param program: name of the executable to find
    :type program: str
    :returns: Full path to the first instance of the executable, or None
    :rtype: str or None
    """
    def is_exe(fpath):
        return os.path.isfile(fpath) and os.access(fpath, os.X_OK)

    fpath, _ = os.path.split(program)
    if fpath:
        if is_exe(program):
            return program
    else:
        for path in os.environ.get('PATH', os.defpath).split(os.pathsep):
            path = path.strip('"')
            exe_file = os.path.join(path, program)
            if is_exe(exe_file):
                return exe_file
    return None
