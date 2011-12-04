#!/usr/bin/env python

from __future__ import print_function
import os, sys, yaml, pprint, em, os.path, datetime, dateutil.tz, platform, time
import subprocess

def parse_options():
    import argparse
    parser = argparse.ArgumentParser(description='Creates a gpb from a catkin project.')
    parser.add_argument('--repo', help='The git build package repo to work with. Will be created if it doesn\'t exist')
    parser.add_argument('--upstream', help='Must be a tar ball of the form <project>_<version>.orig.tar.gz')
    parser.add_argument('--version', help='The upstream version.')
    parser.add_argument('--rosdistro', help='The ros distro. electric, fuerte, galapagos')
    parser.add_argument('--build_path', help='A scratch cmake build path.')
    parser.add_argument('--deb_path', help='Where you are going to put the debs.')
    parser.add_argument('--distros', nargs='+', help='A list of debian distros. Default: %(default)s', default=['lucid', 'maverick', 'natty', 'oneiric'])
    parser.add_argument('--sign', help='Sign with the gpg key.')
    return parser.parse_args()

def repo_call(repo, call):
    print('+' + ' '.join(call))
    r = subprocess.check_output(call, cwd=repo)
    print(r, end='')
    return r

def init_git(repo):
    if not os.path.isdir(repo):
        os.makedirs(repo)
        repo_call(repo, ['git', 'init'])

def create_gbp(repo, upstream):
    upstream = os.path.abspath(upstream)
    return repo_call(repo, ['git', 'import-orig', upstream, '--no-interactive'])

def novel_version(repo, version):
    if not os.path.isdir(repo):
        return True
    tags = repo_call(repo, ['git', 'tag'])
    if version in tags:
        return False
    return True

def import_orig(repo, upstream, version):
    if not novel_version(repo, version):
        print('Warning: removing previous upstream version %(version)s!' % locals(), file=sys.stderr)
        repo_call(repo, ['git', 'tag', '-d', 'upstream/%(version)s' % locals()])
    upstream = os.path.abspath(upstream)
    return repo_call(repo, ['git', 'import-orig', upstream, '--no-interactive'])

def catkin_cmake(repo, rosdistro, build_path):
    repo = os.path.abspath(repo)
    build_path = os.path.abspath(build_path)
    if not os.path.isdir(build_path):
        os.makedirs(build_path)
    CMAKE = '''cmake
-DCMAKE_INSTALL_PREFIX=/opt/ros/%(rosdistro)s
-DCMAKE_PREFIX_PATH=/opt/ros/%(rosdistro)s
-DCATKIN_PACKAGE_PREFIX=ros-%(rosdistro)s-
-DCATKIN_DEB_SNAPSHOTS=YES
%(repo)s''' % locals()
    CMAKE = CMAKE.split('\n')
    repo_call(build_path, CMAKE)

def catkin_gendebian_files(repo, rosdistro, distro, version, build_path, deb_path, sign, stamp):
    signed = '-uc -us'
    if sign:
        signed = '-k%s' % options.sign
    build_path = os.path.abspath(build_path)
    deb_path = os.path.abspath(deb_path)
    target = 'gendebian-files'
    MAKE = '''make
CATKIN_DEBIAN_DISTRIBUTION=%(distro)s
%(target)s''' % locals()
    MAKE = MAKE.split('\n')
    repo_call(build_path, MAKE)
    repo_call(repo, ['git', 'add', 'debian'])
    message = '''+ Creating debian mods for distro: %(distro)s, ros distro: %(rosdistro)s, upstream version: %(version)s
''' % locals()
    tag = '--git-debian-tag=debian/ros_%(rosdistro)s_%(version)s_%(distro)s' % locals()
    repo_call(repo, ['git', 'commit', '-m', message])
    repo_call(repo, ['git', 'buildpackage', '-S', '--git-export-dir=%s' % deb_path, '--git-retag', '--git-tag', '--git-dist=%s' % distro, tag] + signed.split(' '))

if __name__ == '__main__':
    options = parse_options()
    init_git(options.repo)
    stamp = time.strftime('-%Y%m%d-%H%M%z')
    import_orig(options.repo, options.upstream, options.version)
    catkin_cmake(options.repo, options.rosdistro, options.build_path)
    for distro in options.distros:
        catkin_gendebian_files(options.repo,
                               options.rosdistro,
                               distro,
                               options.version,
                               options.build_path,
                               options.deb_path,
                               options.sign,
                               stamp)
