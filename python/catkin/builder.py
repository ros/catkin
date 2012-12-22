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

import io
import multiprocessing
import os
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
    cline = str(sanitize(line))
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
    # Run the command
    proc = subprocess.Popen(
        cmd, cwd=cwd, shell=False,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT
    )
    out = sys.stdout
    if quiet:
        out = io.StringIO()
    while True:
        line = proc.stdout.readline().decode('utf-8')
        if proc.returncode is not None or not line:
            break
        else:
            try:
                out.write(unicode(colorize_line(line)))
            except Exception as e:
                import traceback
                traceback.print_exc()
                print('<caktin_make> color formatting problem: ' + str(e),
                      file=sys.stderr)
    proc.wait()
    if proc.returncode:
        if type(out) == io.StringIO:
            print(out.getvalue())
        raise subprocess.CalledProcessError(proc.returncode, ' '.join(cmd))
    return out.getvalue() if type(out) == io.StringIO else ''


def run_command(cmd, cwd, quiet=False):
    if not quiet:
        subprocess.check_call(cmd, cwd=cwd)
        return ''
    # Run the command
    proc = subprocess.Popen(
        cmd, cwd=cwd, shell=False,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT
    )
    out = io.StringIO()
    while True:
        line = proc.stdout.readline().decode('utf-8')
        if proc.returncode is not None or not line:
            break
        else:
            out.write(line)
    proc.wait()
    if proc.returncode:
        print(out.getvalue())
        raise subprocess.CalledProcessError(proc.returncode, ' '.join(cmd))
    return out.getvalue()

blue_arrow = '@!@{bf}==>@{wf}'


def _check_build_dir(name, spaces):
    package_build_dir = os.path.join(spaces['build'], name)
    if not os.path.exists(package_build_dir):
        cprint(
            blue_arrow + ' Creating build directory: \'' +
            os.path.relpath(package_build_dir, spaces['workspace']) + '\'@|'
        )
        os.mkdir(package_build_dir)
    return package_build_dir


def isolation_print_command(cmd, path=None):
    cprint(
            blue_arrow + " " + sanitize(cmd) + "@|" +
            (" @!@{kf}in@| '@!" + sanitize(path) + "@|'" if path else '')
        )


def build_catkin_package(path, package, spaces, options):
    cprint(
        "Processing @{cf}catkin@| package: '@!@{bf}" +
        package.name + "@|'"
    )

    # Make the build dir
    build_dir = _check_build_dir(package.name, spaces)

    # Check last_env
    if options['last_env'] is not None:
        cprint(
            blue_arrow + " Building with env: " +
            "'{0}'".format(options['last_env'])
        )

    # Check for Makefile and maybe call cmake
    makefile = os.path.join(build_dir, 'Makefile')
    if not os.path.exists(makefile) or options['force_cmake']:
        # Run cmake
        cmake_cmd = [
            'cmake',
            os.path.dirname(package.filename),
            '-DCATKIN_STATIC_ENV=1',
            '-DCATKIN_DEVEL_PREFIX=' + spaces['devel'] + '',
            '-DCMAKE_INSTALL_PREFIX=' + spaces['install'] + ''
        ]
        isolation_print_command(' '.join(cmake_cmd))
        if options['last_env'] is not None:
            cmake_cmd = [options['last_env']] + cmake_cmd
        run_command_colorized(cmake_cmd, build_dir, options['quiet'])
    else:
        print('Makefile exists, skipping explicit cmake invocation...')
        # Check to see if cmake needs to be run via make
        make_check_cmake_cmd = ['make', 'cmake_check_build_system']
        isolation_print_command(' '.join(make_check_cmake_cmd), build_dir)
        if options['last_env'] is not None:
            make_check_cmake_cmd = [options['last_env']] + make_check_cmake_cmd
        run_command_colorized(
            make_check_cmake_cmd, build_dir, options['quiet']
        )

    # Run make
    make_cmd = ['make', '-j' + str(options['jobs'])]
    isolation_print_command(' '.join(make_cmd), build_dir)
    if options['last_env'] is not None:
        make_cmd = [options['last_env']] + make_cmd
    run_command(make_cmd, build_dir, options['quiet'])

    # Make install
    if options['install']:
        make_install_cmd = ['make', 'install']
        isolation_print_command(' '.join(make_install_cmd), build_dir)
        if options['last_env'] is not None:
            make_install_cmd = [options['last_env']] + make_install_cmd
        run_command(make_install_cmd, build_dir, options['quiet'])


def build_cmake_package(path, package, spaces, options):
    # Notify the user that we are processing a plain cmake package
    cprint(
        "Processing @{cf}plain cmake@| package: '@!@{bf}" + package.name +
        "@|'"
    )

    # Make the build dir
    build_dir = _check_build_dir(package.name, spaces)

    # Check last_env
    if options['last_env'] is not None:
        cprint(blue_arrow + " Building with env: " +
               "'{0}'".format(options['last_env']))

    # Check for Makefile and maybe call cmake
    makefile = os.path.join(build_dir, 'Makefile')
    if not os.path.exists(makefile) or options['force_cmake']:
        # Call cmake
        cmake_cmd = [
            'cmake',
            os.path.dirname(package.filename),
            '-DCMAKE_INSTALL_PREFIX=' + spaces['install'] + ''
        ]
        isolation_print_command(' '.join(cmake_cmd))
        if options['last_env'] is not None:
            cmake_cmd = [options['last_env']] + cmake_cmd
        run_command_colorized(cmake_cmd, build_dir, options['quiet'])
    else:
        print('Makefile exists, skipping explicit cmake invocation...')
        # Check to see if cmake needs to be run via make
        make_check_cmake_cmd = ['make', 'cmake_check_build_system']
        isolation_print_command(' '.join(make_check_cmake_cmd), build_dir)
        if options['last_env'] is not None:
            make_check_cmake_cmd = [options['last_env']] + make_check_cmake_cmd
        run_command_colorized(
            make_check_cmake_cmd, build_dir, options['quiet']
        )

    # Run make
    make_cmd = ['make', '-j' + str(options['jobs'])]
    isolation_print_command(' '.join(make_cmd), build_dir)
    if options['last_env'] is not None:
        make_cmd = [options['last_env']] + make_cmd
    run_command(make_cmd, build_dir, options['quiet'])

    # Make install
    make_install_cmd = ['make', 'install']
    isolation_print_command(' '.join(make_install_cmd), build_dir)
    if options['last_env'] is not None:
        make_install_cmd = [options['last_env']] + make_install_cmd
    run_command(make_install_cmd, build_dir, options['quiet'])


def build_package(path, package, spaces, options, number=None, of=None):
    export_tags = [e.tagname for e in package.exports]
    cprint('@!@{gf}==>@| ', end='')
    new_last_env = None
    if 'metapackage' in export_tags:
        cprint("Skipping @!metapackage@|: '@!@{bf}" + package.name + "@|'")
        new_last_env = options['last_env']
    else:
        if 'build_type' in export_tags:
            build_type_tag = [e.content for e in package.exports
                                        if e.tagname == 'build_type'][0]
        else:
            build_type_tag = 'catkin'
        if build_type_tag == 'catkin':
            build_catkin_package(path, package, spaces, options)
            if options['install']:
                new_last_env = os.path.join(
                    spaces['install'],
                    'env_cached.sh'
                )
            else:
                new_last_env = os.path.join(
                    spaces['build'],
                    package.name,
                    'catkin_generated',
                    'env_cached.sh'
                )
        elif build_type_tag == 'cmake':
            build_cmake_package(path, package, spaces, options)
            new_last_env = options['last_env']
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

    # Construct spaces and options dicts
    spaces = {
        'workspace': workspace,
        'build': buildspace,
        'devel': develspace,
        'install': installspace
    }
    options = {
        'install': install,
        'merge': merge,
        'jobs': jobs,
        'force_cmake': force_cmake,
        'quiet': quiet,
        'last_env': None
    }

    # Build packages
    for index, path_package in enumerate(packages):
        path, package = path_package
        if not merge:
            spaces['devel'] = os.path.join(develspace, package.name)
        try:
            options['last_env'] = build_package(
                path, package, spaces, options,
                number=index + 1, of=len(packages)
            )
        except subprocess.CalledProcessError as e:
            cprint(
                '@{rf}@!<==@| ' +
                'Failed to process package \'@!@{bf}' +
                package.name + '@|\': \n  ' +
                str(e)
            )
            sys.exit('Command failed, exiting.')
