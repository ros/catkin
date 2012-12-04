from __future__ import print_function
import multiprocessing
import os
import subprocess
import sys

try:
    from catkin_pkg.topological_order import topological_order
except ImportError as e:
    sys.exit('ImportError: "from catkin_pkg.topological_order import topological_order" failed: %s\nMake sure that you have installed "catkin_pkg", it is up to date and on the PYTHONPATH.' % e)


def build_workspace_in_isolation(sourcespace_dir, install=False, merge=False, force_cmake=False, number_of_threads=None):
    '''
    Runs ``cmake``, ``make`` and optionally ``make install`` for all
    catkin packages in sourcespace_dir.  It creates several folders
    in the current working directory.

    :param sourcespace_dir: folder with catkin packages, ``str``
    :param install: (optional), if True also installs in local install dir, ``bool``
    :param merge: (optional), if True uses a single devel (and install) folder for all packages, ``bool``
    :param force_cmake: (optional), if True calls cmake explicitly for each package, ``bool``
    :param number_of_threads: (optional), the number of thread to use for make (default is the number of CPUs), ``int``
    '''
    sourcespace_dir = os.path.abspath(sourcespace_dir)
    packages = topological_order(sourcespace_dir)
    build_isolated_dir = os.path.abspath('.')

    if not number_of_threads:
        try:
            number_of_threads = multiprocessing.cpu_count()
        except NotImplementedError:
            pass

    last_env_to_source = None
    last_space_dir = None
    for path, package in packages:
        if 'metapackage' in [e.tagname for e in package.exports]:
            print('\n+++ Skipping metapackage "%s"' % path)
            continue
        print('\n+++ Building package "%s"\n' % path)

        package_build_dir = os.path.join(build_isolated_dir, package.name)
        if not os.path.exists(package_build_dir):
            os.mkdir(package_build_dir)

        if not merge:
            package_devel_dir = os.path.join(build_isolated_dir, package.name + '__devel')
        else:
            package_devel_dir = os.path.join(build_isolated_dir, 'devel')

        if install:
            if not merge:
                package_install_dir = os.path.join(build_isolated_dir, package.name + '__install')
            else:
                package_install_dir = os.path.join(build_isolated_dir, 'install')
            if not os.path.exists(package_install_dir):
                os.mkdir(package_install_dir)

        makefile = os.path.join(package_build_dir, 'Makefile')
        if not os.path.exists(makefile) or force_cmake:
            cmd = ['cmake', os.path.dirname(package.filename), '-DCATKIN_STATIC_ENV=1', '-DCATKIN_DEVEL_PREFIX=%s' % package_devel_dir]
            if install:
                cmd.append('-DCMAKE_INSTALL_PREFIX=%s' % package_install_dir)
            _run_command_with_env(cmd, package_build_dir, last_env_to_source)

        cmd = ['make']
        if number_of_threads:
            cmd.append('-j%d' % number_of_threads)
        _run_command_with_env(cmd, package_build_dir, last_env_to_source)

        if not install:
            last_env_to_source = os.path.join(package_build_dir, 'catkin_generated', 'env_cached.sh')
            last_space_dir = package_devel_dir
        else:
            cmd = ['make', 'install']
            _run_command_with_env(cmd, package_build_dir, last_env_to_source)
            last_env_to_source = os.path.join(package_install_dir, 'env_cached.sh')
            last_space_dir = package_install_dir

    if last_space_dir:
        print('\n+++ DONE')
        space_attribute = 'latest' if not merge else 'merged'
        space_type = 'devel space' if not install else 'install space'
        print('\n+++ The %s %s is: %s' % (space_attribute, space_type, last_space_dir))


def _run_command_with_env(cmd, cwd, env_to_source=None):
    if env_to_source is not None:
        cmd = [env_to_source] + cmd
    print('\n    Running command: %s in %s' % (cmd, cwd))
    subprocess.check_call(cmd, cwd=cwd)
