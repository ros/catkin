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

from __future__ import print_function

import copy
import io
import multiprocessing
import os
import re
import stat
import subprocess
import sys

try:
    from catkin_pkg.cmake import configure_file, get_metapackage_cmake_template_path
    from catkin_pkg.packages import find_packages
    from catkin_pkg.topological_order import topological_order_packages
except ImportError as e:
    sys.exit(
        'ImportError: "from catkin_pkg.topological_order import '
        'topological_order" failed: %s\nMake sure that you have installed '
        '"catkin_pkg", it is up to date and on the PYTHONPATH.' % e
    )

from catkin.cmake import get_cmake_path
from catkin.terminal_color import ansi, disable_ANSI_colors, fmt, sanitize


def split_arguments(args, splitter_name, default=None):
    if splitter_name not in args:
        return args, default
    index = args.index(splitter_name)
    return args[0:index], args[index + 1:]


def extract_cmake_and_make_arguments(args):
    args, cmake_args, make_args, _ = _extract_cmake_and_make_arguments(args, extract_catkin_make=False)
    return args, cmake_args, make_args


def extract_cmake_and_make_and_catkin_make_arguments(args):
    return _extract_cmake_and_make_arguments(args, extract_catkin_make=True)


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


def cprint(msg, end=None):
    print(fmt(msg), end=end)


def colorize_line(line):
    cline = sanitize(line)
    cline = cline.replace(
        '-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~',
        '-- @{pf}~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~@|'
    )
    if line.startswith('-- ~~'):
        # -- ~~  -
        cline = cline.replace('~~ ', '@{pf}~~ @|')
        cline = cline.replace(' - ', ' - @!@{bf}')
        cline = cline.replace('(', '@|(')
        cline = cline.replace('(plain cmake)', '@|(@{rf}plain cmake@|)')
        cline = cline.replace('(unknown)', '@|(@{yf}unknown@|)')
    if line.startswith('-- +++'):
        # -- +++ add_subdirectory(package)
        cline = cline.replace('+++', '@!@{gf}+++@|')
        cline = cline.replace('kin package: \'', 'kin package: \'@!@{bf}')
        cline = cline.replace(')', '@|)')
        cline = cline.replace('\'\n', '@|\'\n')
        cline = cline.replace('cmake package: \'', 'cmake package: \'@!@{bf}')
        cline = cline.replace('\'\n', '@|\'\n')
    if line.startswith('-- ==>'):
        cline = cline.replace('-- ==>', '-- @!@{bf}==>@|')
    if line.lower().startswith('warning'):
        # WARNING
        cline = ansi('yf') + cline
    if line.startswith('CMake Warning'):
        # CMake Warning...
        cline = cline.replace('CMake Warning', '@{yf}@!CMake Warning@|')
    if line.startswith('ERROR:'):
        # ERROR:
        cline = cline.replace('ERROR:', '@!@{rf}ERROR:@|')
    if line.startswith('CMake Error'):
        # CMake Error...
        cline = cline.replace('CMake Error', '@{rf}@!CMake Error@|')
    if line.startswith('Call Stack (most recent call first):'):
        # CMake Call Stack
        cline = cline.replace('Call Stack (most recent call first):',
                              '@{cf}@_Call Stack (most recent call first):@|')
    return fmt(cline)


def print_command_banner(cmd, cwd, color):
    if color:
        # Prepare for printing
        cmd_str = sanitize(' '.join(cmd))
        cwd_str = sanitize(cwd)
        # Print command notice
        cprint('@{bf}####')
        cprint('@{bf}#### Running command: @!"%s"@|@{bf} in @!"%s"' % (cmd_str, cwd_str))
        cprint('@{bf}####')
    else:
        print('####')
        print('#### Running command: "%s" in "%s"' % (' '.join(cmd), cwd))
        print('####')


def run_command_colorized(cmd, cwd, quiet=False):
    run_command(cmd, cwd, quiet=quiet, colorize=True)


def run_command(cmd, cwd, quiet=False, colorize=False):
    capture = (quiet or colorize)
    stdout_pipe = subprocess.PIPE if capture else None
    stderr_pipe = subprocess.STDOUT if capture else None
    try:
        proc = subprocess.Popen(
            cmd, cwd=cwd, shell=False,
            stdout=stdout_pipe, stderr=stderr_pipe
        )
    except OSError as e:
        raise OSError("Failed command '%s': %s" % (cmd, e))
    out = io.StringIO() if quiet else sys.stdout
    if capture:
        while True:
            line = unicode(proc.stdout.readline())
            if proc.returncode is not None or not line:
                break
            try:
                line = colorize_line(line) if colorize else line
            except Exception as e:
                import traceback
                traceback.print_exc()
                print('<caktin_make> color formatting problem: ' + str(e),
                      file=sys.stderr)
            out.write(line)
    proc.wait()
    if proc.returncode:
        if quiet:
            print(out.getvalue())
        raise subprocess.CalledProcessError(proc.returncode, ' '.join(cmd))
    return out.getvalue() if quiet else ''

blue_arrow = '@!@{bf}==>@|@!'


def _check_build_dir(name, workspace, buildspace):
    package_build_dir = os.path.join(buildspace, name)
    if not os.path.exists(package_build_dir):
        cprint(
            blue_arrow + ' Creating build directory: \'' +
            os.path.relpath(package_build_dir, workspace) + '\'@|'
        )
        os.mkdir(package_build_dir)
    return package_build_dir


def isolation_print_command(cmd, path=None):
    cprint(
        blue_arrow + " " + sanitize(cmd) + "@|" +
        (" @!@{kf}in@| '@!" + sanitize(path) + "@|'" if path else '')
    )


def get_python_path(path):
    python_path = []
    lib_path = os.path.join(path, 'lib')
    if os.path.exists(lib_path):
        items = os.listdir(lib_path)
        for item in items:
            if os.path.isdir(item) and item.startswith('python'):
                python_items = os.listdir(os.path.join(lib_path, item))
                for py_item in python_items:
                    if py_item in ['dist-packages', 'site-packages']:
                        py_path = os.path.join(lib_path, item, py_item)
                        if os.path.isdir(py_path):
                            python_path.append(py_path)
    return python_path


def handle_make_arguments(input_make_args, force_single_threaded_when_running_tests=False):
    make_args = list(input_make_args)

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


def extract_jobs_flags(mflags):
    regex = r'(?:^|\s)(-?(?:j|l)(?:\s*[0-9]+|\s|$))' + \
            r'|' + \
            r'(?:^|\s)((?:--)?(?:jobs|load-average)(?:(?:=|\s+)[0-9]+|(?:\s|$)))'
    matches = re.findall(regex, mflags) or []
    matches = [m[0] or m[1] for m in matches]
    return ' '.join([m.strip() for m in matches]) if matches else None


def build_catkin_package(
    path, package,
    workspace, buildspace, develspace, installspace,
    install, force_cmake, quiet, last_env, cmake_args, make_args
):
    cprint(
        "Processing @{cf}catkin@| package: '@!@{bf}" +
        package.name + "@|'"
    )

    # Make the build dir
    build_dir = _check_build_dir(package.name, workspace, buildspace)

    # Check last_env
    if last_env is not None:
        cprint(
            blue_arrow + " Building with env: " +
            "'{0}'".format(last_env)
        )

    # Check for Makefile and maybe call cmake
    makefile = os.path.join(build_dir, 'Makefile')
    if not os.path.exists(makefile) or force_cmake:
        package_dir = os.path.dirname(package.filename)
        if not os.path.exists(os.path.join(package_dir, 'CMakeLists.txt')):
            export_tags = [e.tagname for e in package.exports]
            if 'metapackage' not in export_tags:
                print(colorize_line('Error: Package "%s" does not have a CMakeLists.txt file' % package.name))
                sys.exit('Can not build catkin package without CMakeLists.txt file')
            # generate CMakeLists.txt for metpackages without one
            print(colorize_line('Warning: metapackage "%s" should have a CMakeLists.txt file' % package.name))
            cmake_code = configure_file(
                get_metapackage_cmake_template_path(),
                {'name': package.name, 'metapackage_arguments': 'DIRECTORY "%s"' % package_dir})
            cmakelists_txt = os.path.join(build_dir, 'CMakeLists.txt')
            with open(cmakelists_txt, 'w') as f:
                f.write(cmake_code)
            package_dir = build_dir

        # Run cmake
        cmake_cmd = [
            'cmake',
            package_dir,
            '-DCATKIN_DEVEL_PREFIX=' + develspace,
            '-DCMAKE_INSTALL_PREFIX=' + installspace
        ]
        cmake_cmd.extend(cmake_args)
        isolation_print_command(' '.join(cmake_cmd))
        if last_env is not None:
            cmake_cmd = [last_env] + cmake_cmd
        try:
            run_command_colorized(cmake_cmd, build_dir, quiet)
        except subprocess.CalledProcessError as e:
            if os.path.exists(makefile):
                # remove Makefile to force CMake invocation next time
                os.remove(makefile)
            raise
    else:
        print('Makefile exists, skipping explicit cmake invocation...')
        # Check to see if cmake needs to be run via make
        make_check_cmake_cmd = ['make', 'cmake_check_build_system']
        isolation_print_command(' '.join(make_check_cmake_cmd), build_dir)
        if last_env is not None:
            make_check_cmake_cmd = [last_env] + make_check_cmake_cmd
        run_command_colorized(
            make_check_cmake_cmd, build_dir, quiet
        )

    # Run make
    make_cmd = ['make']
    make_cmd.extend(handle_make_arguments(make_args, force_single_threaded_when_running_tests=True))
    isolation_print_command(' '.join(make_cmd), build_dir)
    if last_env is not None:
        make_cmd = [last_env] + make_cmd
    run_command(make_cmd, build_dir, quiet)

    # Make install
    if install:
        make_install_cmd = ['make', 'install']
        isolation_print_command(' '.join(make_install_cmd), build_dir)
        if last_env is not None:
            make_install_cmd = [last_env] + make_install_cmd
        run_command(make_install_cmd, build_dir, quiet)


def build_cmake_package(
    path, package,
    workspace, buildspace, develspace, installspace,
    install, force_cmake, quiet, last_env, cmake_args, make_args
):
    # Notify the user that we are processing a plain cmake package
    cprint(
        "Processing @{cf}plain cmake@| package: '@!@{bf}" + package.name +
        "@|'"
    )

    # Make the build dir
    build_dir = _check_build_dir(package.name, workspace, buildspace)

    # Check last_env
    if last_env is not None:
        cprint(blue_arrow + " Building with env: " +
               "'{0}'".format(last_env))

    # Check for Makefile and maybe call cmake
    makefile = os.path.join(build_dir, 'Makefile')
    install_target = installspace if install else develspace
    if not os.path.exists(makefile) or force_cmake:
        # Call cmake
        cmake_cmd = [
            'cmake',
            os.path.dirname(package.filename),
            '-DCMAKE_INSTALL_PREFIX=' + install_target
        ]
        cmake_cmd.extend(cmake_args)
        isolation_print_command(' '.join(cmake_cmd))
        if last_env is not None:
            cmake_cmd = [last_env] + cmake_cmd
        run_command_colorized(cmake_cmd, build_dir, quiet)
    else:
        print('Makefile exists, skipping explicit cmake invocation...')
        # Check to see if cmake needs to be run via make
        make_check_cmake_cmd = ['make', 'cmake_check_build_system']
        isolation_print_command(' '.join(make_check_cmake_cmd), build_dir)
        if last_env is not None:
            make_check_cmake_cmd = [last_env] + make_check_cmake_cmd
        run_command_colorized(
            make_check_cmake_cmd, build_dir, quiet
        )

    # Run make
    make_cmd = ['make']
    make_cmd.extend(handle_make_arguments(make_args))
    isolation_print_command(' '.join(make_cmd), build_dir)
    if last_env is not None:
        make_cmd = [last_env] + make_cmd
    run_command(make_cmd, build_dir, quiet)

    # Make install
    make_install_cmd = ['make', 'install']
    isolation_print_command(' '.join(make_install_cmd), build_dir)
    if last_env is not None:
        make_install_cmd = [last_env] + make_install_cmd
    run_command(make_install_cmd, build_dir, quiet)

    # If we are installing, and a env.sh exists, don't overwrite it
    if install and os.path.exists(os.path.join(installspace, 'env.sh')):
        return
    cprint(blue_arrow + " Generating an env.sh")
    # Generate env.sh for chaining to catkin packages
    new_env_path = os.path.join(install_target, 'env.sh')
    variables = {
        'SETUP_DIR': install_target,
        'SETUP_FILENAME': 'setup'
    }
    with open(os.path.join(new_env_path), 'w') as f:
        f.write(configure_file(os.path.join(get_cmake_path(), 'templates', 'env.sh.in'), variables))
    os.chmod(new_env_path, stat.S_IXUSR | stat.S_IWUSR | stat.S_IRUSR)

    # Generate setup.sh for chaining to catkin packages
    new_setup_path = os.path.join(install_target, 'setup.sh')
    subs = {}
    subs['cmake_prefix_path'] = install_target + ":"
    subs['ld_path'] = os.path.join(install_target, 'lib') + ":"
    pythonpath = ":".join(get_python_path(install_target))
    if pythonpath:
        pythonpath += ":"
    subs['pythonpath'] = pythonpath
    subs['pkgcfg_path'] = os.path.join(install_target, 'lib', 'pkgconfig')
    subs['pkgcfg_path'] += ":"
    subs['path'] = os.path.join(install_target, 'bin') + ":"
    if not os.path.exists(install_target):
        os.mkdir(install_target)
    with open(new_setup_path, 'w+') as file_handle:
        file_handle.write("""\
#!/usr/bin/env sh
# generated from catkin.builder module

""")
        if last_env is not None:
            last_setup_env = os.path.join(os.path.dirname(last_env), 'setup.sh')
            file_handle.write('. %s\n\n' % last_setup_env)
        file_handle.write("""\
# detect if running on Darwin platform
UNAME=`which uname`
UNAME=`$UNAME`
IS_DARWIN=0
if [ "$UNAME" = "Darwin" ]; then
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


def build_package(
    path, package,
    workspace, buildspace, develspace, installspace,
    install, force_cmake, quiet, last_env, cmake_args, make_args, catkin_make_args,
    number=None, of=None
):
    cprint('@!@{gf}==>@| ', end='')
    new_last_env = get_new_env(package, develspace, installspace, install, last_env)
    build_type = _get_build_type(package)
    if build_type == 'catkin':
        build_catkin_package(
            path, package,
            workspace, buildspace, develspace, installspace,
            install, force_cmake, quiet, last_env, cmake_args, make_args + catkin_make_args
        )
        if not os.path.exists(new_last_env):
            raise RuntimeError(
                "No env.sh file generated at: '" + new_last_env +
                "'\n  This sometimes occurs when a non-catkin package is "
                "interpreted as a catkin package.\n  This can also occur "
                "when the cmake cache is stale, try --force-cmake."
            )
    elif build_type == 'cmake':
        build_cmake_package(
            path, package,
            workspace, buildspace, develspace, installspace,
            install, force_cmake, quiet, last_env, cmake_args, make_args
        )
    else:
        sys.exit('Can not build package with unknown build_type')
    if number is not None and of is not None:
        msg = ' [@{gf}@!' + str(number) + '@| of @!@{gf}' + str(of) + '@|]'
    else:
        msg = ''
    cprint('@{gf}<==@| Finished processing package' + msg + ': \'@{bf}@!' +
           package.name + '@|\'')
    return new_last_env


def get_new_env(package, develspace, installspace, install, last_env):
    new_env = None
    build_type = _get_build_type(package)
    if build_type in ['catkin', 'cmake']:
        new_env = os.path.join(
            installspace if install else develspace,
            'env.sh'
        )
    return new_env


def _get_build_type(package):
    build_type = 'catkin'
    if 'build_type' in [e.tagname for e in package.exports]:
        build_type = [e.content for e in package.exports if e.tagname == 'build_type'][0]
    return build_type


def build_workspace_isolated(
    workspace='.',
    sourcespace=None,
    buildspace=None,
    develspace=None,
    installspace=None,
    merge=False,
    install=False,
    force_cmake=False,
    colorize=True,
    build_packages=None,
    quiet=False,
    cmake_args=None,
    make_args=None,
    catkin_make_args=None
):
    '''
    Runs ``cmake``, ``make`` and optionally ``make install`` for all
    catkin packages in sourcespace_dir.  It creates several folders
    in the current working directory. For non-catkin packages it runs
    ``cmake``, ``make`` and ``make install`` for each, installing it to
    the devel space or install space if the ``install`` option is specified.

    :param workspace: path to the current workspace, ``str``
    :param sourcespace: workspace folder containing catkin packages, ``str``
    :param buildspace: path to build space location, ``str``
    :param develspace: path to devel space location, ``str``
    :param installspace: path to install space (CMAKE_INSTALL_PREFIX), ``str``
    :param merge: if True, build each catkin package into the same
        devel space. does not work with non-catkin packages, ``bool``
    :param install: if True, install all packages to the install space,
        ``bool``
    :param force_cmake: (optional), if True calls cmake explicitly for each
        package, ``bool``
    :param colorize: if True, colorize cmake output and other messages,
        ``bool``
    :param build_packages: specific packages to build (all parent packages
        in the topological order must have been built before), ``str``
    :param quiet: if True, hides some build output, ``bool``
    :param cmake_args: additional arguments for cmake, ``[str]``
    :param make_args: additional arguments for make, ``[str]``
    :param catkin_make_args: additional arguments for make but only for catkin
        packages, ``[str]``
    '''
    if not colorize:
        disable_ANSI_colors()

    # Check workspace existance
    if not os.path.exists(workspace):
        sys.exit("Workspace path '{0}' does not exist.".format(workspace))
    workspace = os.path.abspath(workspace)

    # Check source space existance
    if sourcespace is None:
        ws_sourcespace = os.path.join(workspace, 'src')
        if not os.path.exists(ws_sourcespace):
            sys.exit("Could not find source space: {0}".format(sourcespace))
        sourcespace = ws_sourcespace
    sourcespace = os.path.abspath(sourcespace)
    print('Base path: ' + str(workspace))
    print('Source space: ' + str(sourcespace))

    # Check build space
    if buildspace is None:
        buildspace = os.path.join(workspace, 'build_isolated')
    buildspace = os.path.abspath(buildspace)
    if not os.path.exists(buildspace):
        os.mkdir(buildspace)
    print('Build space: ' + str(buildspace))

    # Check devel space
    if develspace is None:
        develspace = os.path.join(workspace, 'devel_isolated')
    develspace = os.path.abspath(develspace)
    print('Devel space: ' + str(develspace))

    # Check install space
    if installspace is None:
        installspace = os.path.join(workspace, 'install_isolated')
    installspace = os.path.abspath(installspace)
    print('Install space: ' + str(installspace))

    if cmake_args:
        print("Additional CMake Arguments: " + " ".join(cmake_args))
    else:
        cmake_args = []

    if make_args:
        print("Additional make Arguments: " + " ".join(make_args))
    else:
        make_args = []

    if catkin_make_args:
        print("Additional make Arguments for catkin packages: " + " ".join(catkin_make_args))
    else:
        catkin_make_args = []

    # Find packages
    packages = find_packages(sourcespace, exclude_subspaces=True)
    if not packages:
        sys.exit("No packages found in source space: {0}".format(sourcespace))

    # verify that specified package exists in workspace
    if build_packages:
        packages_by_name = {p.name: path for path, p in packages.iteritems()}
        unknown_packages = [p for p in build_packages if p not in packages_by_name]
        if unknown_packages:
            sys.exit('Packages not found in the workspace: %s' % ', '.join(unknown_packages))

    # Report topological ordering
    ordered_packages = topological_order_packages(packages)
    unknown_build_types = []
    msg = []
    msg.append('@{pf}~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~' + ('~' * len(str(len(ordered_packages)))))
    msg.append('@{pf}~~@|  traversing %d packages in topological order:' % len(ordered_packages))
    for path, package in ordered_packages:
        export_tags = [e.tagname for e in package.exports]
        if 'build_type' in export_tags:
            build_type_tag = [e.content for e in package.exports if e.tagname == 'build_type'][0]
        else:
            build_type_tag = 'catkin'
        if build_type_tag == 'catkin':
            msg.append('@{pf}~~@|  - @!@{bf}' + package.name + '@|')
        elif build_type_tag == 'cmake':
            msg.append(
                '@{pf}~~@|  - @!@{bf}' + package.name + '@|' +
                ' (@!@{cf}plain cmake@|)'
            )
        else:
            msg.append(
                '@{pf}~~@|  - @!@{bf}' + package.name + '@|' +
                ' (@{rf}unknown@|)'
            )
            unknown_build_types.append(package)
    msg.append('@{pf}~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~' + ('~' * len(str(len(ordered_packages)))))
    for index in range(len(msg)):
        msg[index] = fmt(msg[index])
    print('\n'.join(msg))

    # Error if there are packages with unknown build_types
    if unknown_build_types:
        print(colorize_line('Error: Packages with unknown build types exist'))
        sys.exit('Can not build workspace with packages of unknown build_type')

    # Check to see if the workspace has changed
    if not force_cmake:
        force_cmake, install_toggled = cmake_input_changed(
            packages,
            buildspace,
            install=install,
            cmake_args=cmake_args,
            filename='catkin_make_isolated'
        )
        if force_cmake:
            print('The packages or cmake arguments have changed, forcing cmake invocation')
        elif install_toggled:
            print('The install argument has been toggled, forcing cmake invocation on plain cmake package')

    # Build packages
    original_develspace = copy.deepcopy(develspace)
    last_env = None
    for index, path_package in enumerate(ordered_packages):
        path, package = path_package
        if not merge:
            develspace = os.path.join(original_develspace, package.name)
        if not build_packages or package.name in build_packages:
            try:
                export_tags = [e.tagname for e in package.exports]
                is_cmake_package = 'cmake' in [e.content for e in package.exports if e.tagname == 'build_type']
                last_env = build_package(
                    path, package,
                    workspace, buildspace, develspace, installspace,
                    install, force_cmake or (install_toggled and is_cmake_package),
                    quiet, last_env, cmake_args, make_args, catkin_make_args,
                    number=index + 1, of=len(ordered_packages)
                )
            except Exception as e:
                import traceback
                traceback.print_exc()
                cprint(
                    '@{rf}@!<==@| ' +
                    'Failed to process package \'@!@{bf}' +
                    package.name + '@|\': \n  ' +
                    ('KeyboardInterrupt' if isinstance(e, KeyboardInterrupt)
                        else str(e))
                )
                if isinstance(e, subprocess.CalledProcessError):
                    cmd = ' '.join(e.cmd) if isinstance(e.cmd, list) else e.cmd
                    print(fmt("\n@{rf}Reproduce this error by running:"))
                    print(fmt("@{gf}@!==> @|") + cmd + "\n")
                sys.exit('Command failed, exiting.')
        else:
            cprint("Skipping package: '@!@{bf}" + package.name + "@|'")
            last_env = get_new_env(package, develspace, installspace, install, last_env)

    # Provide a top level devel space environment setup script
    if not merge and not build_packages and os.path.exists(original_develspace):
        # generate env.sh and setup.sh which relay to last devel space
        generated_env = os.path.join(original_develspace, 'env.sh')
        with open(generated_env, 'w') as f:
            f.write("""\
#!/usr/bin/env sh
# generated from catkin.builder module

{0} "$@"
""".format(os.path.join(develspace, 'env.sh')))
        os.chmod(generated_env, stat.S_IXUSR | stat.S_IWUSR | stat.S_IRUSR)
        with open(os.path.join(original_develspace, 'setup.sh'), 'w') as f:
            f.write("""\
#!/usr/bin/env sh
# generated from catkin.builder module

. "{0}/setup.sh"
""".format(develspace))
        # generate setup.bash and setup.zsh for convenience
        variables = {'SETUP_DIR': original_develspace}
        with open(os.path.join(original_develspace, 'setup.bash'), 'w') as f:
            f.write(configure_file(os.path.join(get_cmake_path(), 'templates', 'setup.bash.in'), variables))
        with open(os.path.join(original_develspace, 'setup.zsh'), 'w') as f:
            f.write(configure_file(os.path.join(get_cmake_path(), 'templates', 'setup.zsh.in'), variables))


def cmake_input_changed(packages, build_path, install=None, cmake_args=None, filename='catkin_make'):
    # get current input
    package_paths = os.pathsep.join(sorted(packages.keys()))
    cmake_args = ' '.join(cmake_args) if cmake_args else ''

    # file to store current input
    changed = False
    install_toggled = False
    input_filename = os.path.join(build_path, '%s.cache' % filename)
    if not os.path.exists(input_filename):
        changed = True
    else:
        # compare with previously stored input
        with open(input_filename, 'r') as f:
            previous_package_paths = f.readline().rstrip()
            previous_cmake_args = f.readline().rstrip()
            previous_install = f.readline().rstrip() == str(True)
        if package_paths != previous_package_paths:
            changed = True
        if cmake_args != previous_cmake_args:
            changed = True
        if install is not None and install != previous_install:
            install_toggled = True

    # store current input for next invocation
    with open(input_filename, 'w') as f:
        f.write('%s\n%s\n%s' % (package_paths, cmake_args, install))

    return changed, install_toggled
