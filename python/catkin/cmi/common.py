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


def create_build_space(buildspace, package_name):
    package_build_dir = os.path.join(buildspace, package_name)
    if not os.path.exists(package_build_dir):
        os.makedirs(package_build_dir)
    return package_build_dir


def extract_jobs_flags(mflags):
    regex = r'(?:^|\s)(-?(?:j|l)(?:\s*[0-9]+|\s|$))' + \
            r'|' + \
            r'(?:^|\s)((?:--)?(?:jobs|load-average)(?:(?:=|\s+)[0-9]+|(?:\s|$)))'
    matches = re.findall(regex, mflags) or []
    matches = [m[0] or m[1] for m in matches]
    return ' '.join([m.strip() for m in matches]) if matches else None


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


def log(*args, **kwargs):
    if 'end_with_escape' in kwargs and kwargs['end_with_escape']:
        args = list(args)
        args.append('\033[0m')
        del kwargs['end_with_escape']
    print(*args, **kwargs)


def terminal_width():
    """Estimate the width of the terminal"""
    width = 0
    try:
        import struct
        import fcntl
        import termios
        s = struct.pack('HHHH', 0, 0, 0, 0)
        x = fcntl.ioctl(1, termios.TIOCGWINSZ, s)
        width = struct.unpack('HHHH', x)[1]
    except IOError:
        pass
    if width <= 0:
        try:
            width = int(os.environ['COLUMNS'])
        except:
            pass
    if width <= 0:
        width = 80

    return width


def remove_ansi_escape(string):
    ansi_escape = re.compile(r'\x1b[^m]*m')
    return ansi_escape.sub('', string)


def wide_log(msg, **kwargs):
    width = terminal_width()
    if len(msg) < width:
        log(msg + (' ' * (width - len(remove_ansi_escape(msg)) - 1)), **kwargs)
    else:
        log(msg, **kwargs)
