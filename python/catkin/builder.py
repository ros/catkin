from __future__ import print_function
import os
import subprocess

from catkin_pkg.topological_order import topological_order


def build_workspace_in_isolation(sourcespace_dir, buildspace_parent_dir):
    sourcespace_dir = os.path.abspath(sourcespace_dir)
    packages = topological_order(sourcespace_dir)

    buildspace_dir = os.path.abspath(os.path.join(buildspace_parent_dir, 'build_isolated'))
    if not os.path.exists(buildspace_dir):
        os.makedirs(buildspace_dir)

    last_package_build_dir = None
    for path, package in packages:
        if 'metapackage' in [e.tagname for e in package.exports]:
            print('\n+++ Skipping metapackage "%s"' % path)
            continue
        print('\n+++ Building package "%s"\n' % path)
        package_build_dir = os.path.join(buildspace_dir, package.name)
        if not os.path.exists(package_build_dir):
            os.mkdir(package_build_dir)

        makefile = os.path.join(package_build_dir, 'Makefile')
        if not os.path.exists(makefile):
            cmd = ['cmake', os.path.dirname(package.filename), '-DCATKIN_STATIC_ENV=1']
            _run_command_with_env(cmd, package_build_dir, last_package_build_dir)

        cmd = ['make', '-j8']
        _run_command_with_env(cmd, package_build_dir, last_package_build_dir)

        last_package_build_dir = package_build_dir


def _run_command_with_env(cmd, cwd, last_package_build_dir=None):
    if last_package_build_dir is not None:
        #last_env_sh = os.path.join(last_package_build_dir, 'buildspace', 'env.sh')
        last_env_sh = os.path.join(last_package_build_dir, 'catkin_generated', 'env_cached.sh')
        cmd = [last_env_sh] + cmd
    print('\n    Running command: %s in %s' % (cmd, cwd))
    subprocess.check_call(cmd, cwd=cwd)
