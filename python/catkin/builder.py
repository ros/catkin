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
import stat
import subprocess
import sys

try:
    from catkin_pkg.topological_order import topological_order
except ImportError as e:
    sys.exit(
        'ImportError: "from catkin_pkg.topological_order import '
        'topological_order" failed: %s\nMake sure that you have installed '
        '"catkin_pkg", it is up to date and on the PYTHONPATH.' % e
    )

from catkin.terminal_color import ansi, disable_ANSI_colors, fmt, sanitize


def cprint(msg, end=None):
    print(fmt(msg), end=end)


def colorize_line(line):
    cline = sanitize(line)
    cline = cline.replace(
        '-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~',
        '-- @{pf}~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~@|'
    )
    if line.startswith('-- ~~'):
        # -- ~~  -
        cline = cline.replace('~~ ', '@{pf}~~ @|')
        cline = cline.replace(' - ', ' - @!@{bf}')
        cline = cline.replace('(', '@|(')
        cline = cline.replace('(metapackage)', '@|(@{cf}metapackage@|)')
        cline = cline.replace('(plain cmake)', '@|(@{rf}plain cmake@|)')
        cline = cline.replace('(unknown)', '@|(@{yf}unknown@|)')
    if line.startswith('-- +++'):
        # -- +++ add_subdirectory(package)
        cline = cline.replace('+++', '@!@{gf}+++@|')
        cline = cline.replace('kin package: \'', 'kin package: \'@!@{bf}')
        cline = cline.replace(')', '@|)')
        cline = cline.replace('metapackage: \'', 'metapackage: \'@!@{bf}')
        cline = cline.replace('\'\n', '@|\'\n')
        cline = cline.replace('cmake package: \'', 'cmake package: \'@!@{bf}')
        cline = cline.replace('\'\n', '@|\'\n')
    if line.startswith('-- ==>'):
        cline = cline.replace('-- ==>', '-- @!@{bf}==>@|')
    if line.startswith('WARNING'):
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
        cprint('@{bf}#### Running command: @!"%s"@|@{bf} in @!"%s"' % \
                  (cmd_str, cwd_str))
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
            line = proc.stdout.readline().decode('utf8', 'replace')
            line = unicode(line)
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

blue_arrow = '@!@{bf}==>@{wf}'


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


def build_catkin_package(
    path, package,
    workspace, buildspace, develspace, installspace,
    install, jobs, force_cmake, quiet, last_env
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
        # Run cmake
        cmake_cmd = [
            'cmake',
            os.path.dirname(package.filename),
            '-DCATKIN_DEVEL_PREFIX=' + develspace,
            '-DCMAKE_INSTALL_PREFIX=' + installspace
        ]
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
    make_cmd = ['make', '-j' + str(jobs)]
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
    install, jobs, force_cmake, quiet, last_env
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
    make_cmd = ['make', '-j' + str(jobs)]
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

    # If we are installing, and a env_cached.sh exists, don't overwrite it
    if install and os.path.exists(os.path.join(installspace, 'env_cached.sh')):
        return
    cprint(blue_arrow + " Generating an env_cached.sh")
    # Generate basic env.sh for chaining to catkin packages
    last_env = os.path.join(os.path.dirname(last_env), 'setup.sh')
    new_env_path = os.path.join(install_target, 'env.sh')
    cmake_prefix_path = install_target + ":"
    ld_path = os.path.join(install_target, 'lib') + ":"
    pythonpath = ":".join(get_python_path(install_target))
    if pythonpath:
        pythonpath += ":"
    pkgcfg_path = os.path.join(install_target, 'lib', 'pkgconfig') + ":"
    path = os.path.join(install_target, 'bin') + ":"
    with open(new_env_path, 'w+') as file_handle:
        file_handle.write("""\
#!/bin/sh

# Autogenerated from caktin.builder module

. {last_env}

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

exec "$@"
""".format(**locals())
        )
    os.chmod(new_env_path, stat.S_IXUSR | stat.S_IWUSR | stat.S_IRUSR)


def build_package(
    path, package,
    workspace, buildspace, develspace, installspace,
    install, jobs, force_cmake, quiet, last_env,
    number=None, of=None
):
    export_tags = [e.tagname for e in package.exports]
    cprint('@!@{gf}==>@| ', end='')
    new_last_env = None
    if 'metapackage' in export_tags:
        cprint("Skipping @!metapackage@|: '@!@{bf}" + package.name + "@|'")
        new_last_env = last_env
    else:
        if 'build_type' in export_tags:
            build_type_tag = [e.content for e in package.exports
                                        if e.tagname == 'build_type'][0]
        else:
            build_type_tag = 'catkin'
        if build_type_tag == 'catkin':
            build_catkin_package(
                path, package,
                workspace, buildspace, develspace, installspace,
                install, jobs, force_cmake, quiet, last_env
            )
            if install:
                new_last_env = os.path.join(
                    installspace,
                    'env.sh'
                )
            else:
                new_last_env = os.path.join(
                    develspace,
                    'env.sh'
                )
            if not os.path.exists(new_last_env):
                raise RuntimeError(
                    "No env_cached.sh file generated at: " + new_last_env +
                    "\n  This sometimes occurs when a non-catkin package is "
                    "interpreted as a catkin package."
                )
        elif build_type_tag == 'cmake':
            build_cmake_package(
                path, package,
                workspace, buildspace, develspace, installspace,
                install, jobs, force_cmake, quiet, last_env
            )
            if install:
                new_last_env = os.path.join(
                    installspace,
                    'env.sh'
                )
            else:
                new_last_env = os.path.join(
                    develspace,
                    'env.sh'
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


def build_workspace_isolated(
    workspace='.',
    sourcespace=None,
    buildspace=None,
    develspace=None,
    installspace=None,
    merge=False,
    install=False,
    jobs=None,
    force_cmake=False,
    colorize=True,
    quiet=False
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
    :param jobs: number of parallel build jobs to run (make -jN), ``int``
    :param force_cmake: (optional), if True calls cmake explicitly for each
        package, ``bool``
    :param colorize: if True, colorize cmake output and other messages,
        ``bool``
    :param quiet: if True, hides some build output, ``bool``
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

    # Check jobs
    if not jobs:
        try:
            jobs = multiprocessing.cpu_count()
        except NotImplementedError:
            jobs = 1
    jobs = int(jobs)

    # Find packages
    packages = topological_order(sourcespace)
    if not packages:
        sys.exit("No packages found in source space: {0}".format(sourcespace))

    # Report topological ordering
    unknown_build_types = []
    msg = []
    msg.append('@{pf}~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
    msg.append('@{pf}~~@|  traversing packages in topological order:')
    for path, package in packages:
        export_tags = [e.tagname for e in package.exports]
        if 'metapackage' in export_tags:
            msg.append(
                '@{pf}~~@|  - @!@{bf}' + package.name + '@|' +
                ' (@{cf}metapackage@|)'
            )
        else:
            if 'build_type' in export_tags:
                build_type_tag = [e.content for e in package.exports
                                            if e.tagname == 'build_type'][0]
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
    msg.append('@{pf}~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
    for index in range(len(msg)):
        msg[index] = fmt(msg[index])
    print('\n'.join(msg))

    # Error if there are packages with unknown build_types
    if unknown_build_types:
        print(colorize_line('Error: Packages with unknown build types exist'))
        sys.exit('Can not build workspace with packages of unknown build_type')

    # Build packages
    original_develspace = copy.deepcopy(develspace)
    last_env = None
    for index, path_package in enumerate(packages):
        path, package = path_package
        if not merge:
            develspace = os.path.join(original_develspace, package.name)
        try:
            last_env = build_package(
                path, package,
                workspace, buildspace, develspace, installspace,
                install, jobs, force_cmake, quiet, last_env,
                number=index + 1, of=len(packages)
            )
        except (subprocess.CalledProcessError, KeyboardInterrupt) as e:
            cprint(
                '@{rf}@!<==@| ' +
                'Failed to process package \'@!@{bf}' +
                package.name + '@|\': \n  ' +
                (str(e) if isinstance(e, KeyboardInterrupt)
                        else 'KeyboardInterrupt')
            )
            sys.exit('Command failed, exiting.')
