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
import os
import re
import sys

from multiprocessing import cpu_count

if os.name == 'nt':
    import catkin.cmi.run_windows as run
else:
    import catkin.cmi.run_unix as run

from catkin.cmi.color import clr

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
    """Creates a build space, if it does not already exist, in the build space

    :param buildspace: folder in which packages are built
    :type buildspace: str
    :param package_name: name of the package this build space is for
    :type package_name: str
    :returns: package specific build directory
    :rtype: str
    """
    package_build_dir = os.path.join(buildspace, package_name)
    if not os.path.exists(package_build_dir):
        os.makedirs(package_build_dir)
    return package_build_dir


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

    def split_arguments(args, splitter_name):
        if splitter_name not in args:
            return args, None
        index = args.index(splitter_name)
        return args[0:index], args[index + 1:]

    for index in reversed(sorted(arg_indexes.keys())):
        arg_type = arg_indexes[index]
        args, specific_args = split_arguments(args, arg_type)
        arg_types[arg_type].extend(specific_args)

    # classify -D* and -G* arguments as cmake specific arguments
    implicit_cmake_args = [a for a in args if a.startswith('-D') or a.startswith('-G')]
    args = [a for a in args if a not in implicit_cmake_args]

    return args, implicit_cmake_args + cmake_args, make_args, catkin_make_args


def extract_cmake_and_make_and_catkin_make_arguments(args):
    """Extracts cmake, make, and catkin specific make arguments from given system arguments

    :param args: system arguments from which special arguments need to be extracted
    :type args: list
    :returns: tuple of separate args, cmake_args, make args, and catkin make args
    :rtype: tuple
    """
    return _extract_cmake_and_make_arguments(args, extract_catkin_make=True)


def extract_cmake_and_make_arguments(args):
    """Extracts cmake and make arguments from the given system arguments

    :param args: system arguments from which special arguments need to be extracted
    :param args: list
    :returns: tuple of separate args, cmake_args, and make_args
    :rtype: tuple
    """
    args, cmake_args, make_args, _ = _extract_cmake_and_make_arguments(args, extract_catkin_make=False)
    return args, cmake_args, make_args


def extract_jobs_flags(mflags):
    """Extracts make job flags from a list of other make flags, i.e. -j8 -l8

    :param mflags: string of space separated make arguments
    :type mflags: str
    :returns: space separated list of make jobs flags
    :rtype: str
    """
    regex = r'(?:^|\s)(-?(?:j|l)(?:\s*[0-9]+|\s|$))' + \
            r'|' + \
            r'(?:^|\s)((?:--)?(?:jobs|load-average)(?:(?:=|\s+)[0-9]+|(?:\s|$)))'
    matches = re.findall(regex, mflags) or []
    matches = [m[0] or m[1] for m in matches]
    return ' '.join([m.strip() for m in matches]) if matches else None


def format_time_delta(delta):
    """Formats a given time delta, in seconds, into a day-hour-minute-second string

    Seconds are limited to one decimal point accuracy. Days, hours, and minutes
    are not printed unless required.

    Examples:
        1.45        => 1.4 seconds
        61.45       => 1 minute and 1.4 seconds
        121.45      => 2 minutes and 1.4 seconds
        3721.45     => 1 hour 2 minutes and 1.4 seconds
        7321.45     => 2 hours 2 minutes and 1.4 seconds
        93821.45    => 1 days, 2 hours 2 minutes and 1.4 seconds

    :param delta: time delta to format, in seconds
    :type delta: float
    :returns: formatted time string
    :rtype: str
    """
    days = "0"
    date_str = str(datetime.timedelta(seconds=delta))
    if ', ' in date_str:
        days, date_str = date_str.split(', ')
    hours, minutes, seconds = date_str.split(':')
    msg = "" if int(days.split(' ')[0]) == 0 else days + " "
    msg += "" if int(hours) == 0 else (hours + " hour{0} ".format('' if int(hours) <= 1 else 's'))
    msg += "" if int(minutes) == 0 else ("{0} minute{1} and ".format(int(minutes), '' if int(minutes) <= 1 else 's'))
    msg += "{0:.1f}".format(float(seconds))
    msg += " seconds"
    return msg


def format_time_delta_short(delta):
    """Formats a given time delta, in seconds, into a short day-hour-minute-second string

    Seconds are limited to one decimal point accuracy. Days, hours, and minutes
    are not printed unless required.

    Examples:
        1.45        => 1.4
        61.45       => 1:01.4
        121.45      => 2:01.4
        3721.45     => 1:02:01.4
        7321.45     => 2:02:01.4
        93821.45    => 1 days, 2:02:01.4

    :param delta: time delta to format, in seconds
    :type delta: float
    :returns: formatted time string
    :rtype: str
    """
    days = "0"
    date_str = str(datetime.timedelta(seconds=delta))
    if ', ' in date_str:
        days, date_str = date_str.split(', ')
    hours, minutes, seconds = date_str.split(':')
    msg = "" if int(days.split(' ')[0]) == 0 else days + " "
    msg += "" if int(hours) == 0 else (hours + ":")
    msg += "" if int(minutes) == 0 else (minutes + ":")
    msg += ("{0:.1f}" if int(minutes) == 0 else "{0:04.1f}").format(float(seconds))
    return msg


def handle_make_arguments(input_make_args, force_single_threaded_when_running_tests=False):
    """Special handling for make arguments.

    If force_single_threaded_when_running_tests is True, jobs flags are
    replaced with -j1, because tests cannot handle parallelization.

    If no job flags are present and there are none in the MAKEFLAGS environment
    variable, then make flags are set to the cpu_count, e.g. -j4 -l4.

    :param input_make_args: list of make arguments to be handled
    :type input_make_args: list
    :param force_single_threaded_when_running_tests: self explanatory
    :type force_single_threaded_when_running_tests: bool
    :returns: copied list of make arguments, potentially with some modifications
    :rtype: list
    """
    make_args = list(input_make_args)

    if force_single_threaded_when_running_tests:
        # force single threaded execution when running test since rostest does not support multiple parallel runs
        run_tests = [a for a in make_args if a.startswith('run_tests')]
        if run_tests:
            wide_log('Forcing "-j1" for running unit tests.')
            make_args.append('-j1')

    # If no -j/--jobs/-l/--load-average flags are in make_args
    if not extract_jobs_flags(' '.join(make_args)):
        # If -j/--jobs/-l/--load-average are in MAKEFLAGS
        if 'MAKEFLAGS' in os.environ and extract_jobs_flags(os.environ['MAKEFLAGS']):
            # Do not extend make arguments, let MAKEFLAGS set things
            pass
        else:
            # Else extend the make_arguments to include some jobs flags
            # Use the number of CPU cores
            try:
                jobs = cpu_count()
                make_args.append('-j{0}'.format(jobs))
                make_args.append('-l{0}'.format(jobs))
            except NotImplementedError:
                # If the number of cores cannot be determined, do not extend args
                pass
    return make_args


def get_build_type(package):
    """Returns the build type for a given package

    :param package: package object
    :type package: :py:class:`catkin_pkg.package.Package`
    :returns: build type of the package, e.g. 'catkin' or 'cmake'
    :rtype: str
    """
    export_tags = [e.tagname for e in package.exports]
    if 'build_type' in export_tags:
        build_type_tag = [e.content for e in package.exports if e.tagname == 'build_type'][0]
    else:
        build_type_tag = 'catkin'
    return build_type_tag


def get_python_install_dir():
    """Returns the same value as the CMake variable PYTHON_INSTALL_DIR

    The PYTHON_INSTALL_DIR variable is normally set from the CMake file:

        catkin/cmake/python.cmake

    :returns: Python install directory for the system Python
    :rtype: str
    """
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
    """Returns cached or calculated recursive build dependes for a given package

    If the recursive build depends for this package and this set of workspace
    packages has already been calculated, the cached results are returned.

    :param package: package for which the recursive depends should be calculated
    :type package: :py:class:`catkin_pkg.package.Package`
    :param workspace_packages: packages in the workspace, keyed by name, with
        value being a tuple of package path and package object
    :type workspace_packages: dict(package_name, tuple(package path,
        :py:class:`catkin_pkg.package.Package`))
    :returns: list of package path, package object tuples which are the
        recursive build depends for the given package
    :rtype: list(tuple(package path, :py:class:`catkin_pkg.package.Package`))
    """
    workspace_key = ','.join([pkg.name for pth, pkg in workspace_packages])
    if workspace_key not in __recursive_depends_cache:
        __recursive_depends_cache[workspace_key] = {}
    cache = __recursive_depends_cache[workspace_key]
    if package.name not in cache:
        cache[package.name] = get_recursive_build_depends_in_workspace(package, workspace_packages)
    __recursive_depends_cache[workspace_key] = cache
    return __recursive_depends_cache[workspace_key][package.name]


def get_recursive_build_depends_in_workspace(package, ordered_packages):
    """Calculates the recursive build dependencies of a package which are also in the ordered_packages

    :param package: package for which the recursive depends should be calculated
    :type package: :py:class:`catkin_pkg.package.Package`
    :param workspace_packages: packages in the workspace, keyed by name, with
        value being a tuple of package path and package object
    :type workspace_packages: dict(package_name, tuple(package path,
        :py:class:`catkin_pkg.package.Package`))
    :returns: list of package path, package object tuples which are the
        recursive build depends for the given package
    :rtype: list(tuple(package path, :py:class:`catkin_pkg.package.Package`))
    """
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


def is_tty(stream):
    """Returns True if the given stream is a tty, else False"""
    return hasattr(stream, 'isatty') and stream.isatty()


def log(*args, **kwargs):
    """Wrapper for print, allowing for special handling where necessary"""
    if 'end_with_escape' not in kwargs or kwargs['end_with_escape'] is True:
        args = list(args)
        escape_reset = clr('@|')
        if escape_reset:
            args.append(escape_reset)
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
        return 80

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
    try:
        return terminal_width_windows() if os.name == 'nt' else terminal_width_linux()
    except ValueError:
        # Failed to get the width, use the default 80
        return 80

_ansi_escape = re.compile(r'\x1b[^m]*m')


def remove_ansi_escape(string):
    """Removes any ansi escape sequences from a string and returns it"""
    global _ansi_escape
    return _ansi_escape.sub('', string)


def slice_to_printed_length(string, length):
    """Truncates a string, which may contain non-printable characters, to a printed length

    For example:

        msg = '\033[32mfoo\033[31mbar\033[0m'

    has a length of 20, but a printed length of 6. If you wanted to truncate the
    printed string to 4, then printing ``msg[4]`` would not provide the desired
    result. Instead the actual slice index must consider the non-printable
    characters.

    :param string: string to be truncated
    :type string: str
    :param length: printed length of the resulting string
    :type length: int
    :returns: truncated string
    :rtype: str
    """
    global _ansi_escape
    lookup_array = []
    current_index = 0
    matches = list(_ansi_escape.finditer(string))
    for m in matches:
        for x in range(m.start() - current_index):
            lookup_array.append(current_index)
            current_index += 1
        current_index += len(m.group())
    if not matches:
        # If no matches, then set the lookup_array to a plain range
        lookup_array = range(len(string))
    return string[:lookup_array[length]] + clr('@|')


def __wide_log(msg, **kwargs):
    width = terminal_width()
    rhs = ''
    if 'rhs' in kwargs:
        rhs = ' ' + kwargs['rhs']
        del kwargs['rhs']
    if rhs:
        kwargs['truncate'] = True
    rhs_len = len(remove_ansi_escape(rhs))
    msg_len = len(remove_ansi_escape(msg))
    if 'truncate' in kwargs:
        if kwargs['truncate'] and msg_len >= width - 1:
            msg = slice_to_printed_length(msg, width - rhs_len - 4) + '...'
            msg_len = len(remove_ansi_escape(msg))
        del kwargs['truncate']
    if msg_len + rhs_len < width:
        log(msg + (' ' * (width - msg_len - rhs_len - 1)) + rhs, **kwargs)
    else:
        log(msg, **kwargs)

wide_log_fn = __wide_log


def disable_wide_log():
    """Disables wide logging globally

    :see: :py:func:`wide_log`
    """
    global wide_log_fn

    def disabled_wide_log(msg, **kwargs):
        if 'rhs' in kwargs:
            del kwargs['rhs']
        if 'truncate' in kwargs:
            del kwargs['truncate']
        log(msg, **kwargs)

    wide_log_fn = disabled_wide_log


def wide_log(msg, **kwargs):
    """Prints a message to the screen, filling the remainder of the screen with spaces

    This is useful for printing lines which will completely overwrite previous
    content printed with a carriage return at the end.

    If the message is wider than the terminal, then no filling is done.

    The wide logging can be disabled with :py:func:`disable_wide_log`, in order
    to prevent queries to the terminal width, which is useful when output is
    not to a terminal, like when being used with Continuous Integration.

    Truncating and right hand side messages are disabled when wide_log is
    disabled as well.

    When a right hand side message is given, it implies truncate is True.

    :param msg: message to be printed
    :type msg: str
    :param rhs: message to print at the right hand side of the screen
    :type rhs: str
    :param truncate: If True, messages wider the then terminal will be truncated
    :type truncate: bool
    """
    global wide_log_fn
    wide_log_fn(msg, **kwargs)


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
