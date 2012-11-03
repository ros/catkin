from __future__ import print_function
import os
import subprocess

from catkin_pkg.topological_order import topological_order


def build_workspace_in_isolation(sourcespace_dir, buildspace_parent_dir, install=False):
    sourcespace_dir = os.path.abspath(sourcespace_dir)
    packages = topological_order(sourcespace_dir)

    buildspace_dir = os.path.abspath(os.path.join(buildspace_parent_dir, 'build_isolated'))
    if not os.path.exists(buildspace_dir):
        os.makedirs(buildspace_dir)

    last_env_to_source = None
    last_space_dir = None
    for path, package in packages:
        if 'metapackage' in [e.tagname for e in package.exports]:
            print('\n+++ Skipping metapackage "%s"' % path)
            continue
        print('\n+++ Building package "%s"\n' % path)
        package_build_dir = os.path.join(buildspace_dir, package.name)
        if not os.path.exists(package_build_dir):
            os.mkdir(package_build_dir)
        if install:
            package_install_dir = package_build_dir + '__install'
            if not os.path.exists(package_install_dir):
                os.mkdir(package_install_dir)

        makefile = os.path.join(package_build_dir, 'Makefile')
        if not os.path.exists(makefile):
            cmd = ['cmake', os.path.dirname(package.filename), '-DCATKIN_STATIC_ENV=1']
            if install:
                cmd.append('-DCMAKE_INSTALL_PREFIX=%s' % package_install_dir)
            _run_command_with_env(cmd, package_build_dir, last_env_to_source)

        cmd = ['make', '-j8']
        _run_command_with_env(cmd, package_build_dir, last_env_to_source)

        if not install:
            last_env_to_source = os.path.join(package_build_dir, 'catkin_generated', 'env_cached.sh')
            last_space_dir = os.path.join(package_build_dir, 'buildspace')
        else:
            cmd = ['make', 'install']
            _run_command_with_env(cmd, package_build_dir, last_env_to_source)
            last_env_to_source = os.path.join(package_install_dir, 'env_cached.sh')
            last_space_dir = package_install_dir

    if last_space_dir:
        print('\n+++ DONE')
        print('\n+++ The latest overlay is: %s' % last_space_dir)


def _run_command_with_env(cmd, cwd, env_to_source=None):
    if env_to_source is not None:
        cmd = [env_to_source] + cmd
    print('\n    Running command: %s in %s' % (cmd, cwd))
    subprocess.check_call(cmd, cwd=cwd)
