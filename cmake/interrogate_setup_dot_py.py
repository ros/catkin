#!/usr/bin/env python

from __future__ import print_function
import os
import sys

from argparse import ArgumentParser

def samefile(path1, path2):
    """
    This is a cross-platform implementation for posixpath's
    os.path.samefile.
    """
    try:
        result = os.path.samefile(path1, path2)
    except AttributeError:
        # This is a rather naive implementation.
        # It will break except in the simplest of cases
        # but suffices for catkin on windows.
        result = os.path.normcase(os.path.normpath(path1)) == \
                 os.path.normcase(os.path.normpath(path2))
    return result


def generate_cmake_file(package_name, version, scripts, package_dir, pkgs):
    """
    Generates lines to add to a cmake file which will set variables
    :param version: str, format 'int.int.int'
    :param scripts: [list of str]: relative paths to scripts
    :param package_dir: {modulename: path}
    :pkgs: [list of str] python_packages declared in catkin package
    """
    result = []
    result.append(r'set(%s_VERSION "%s")' % (package_name, version))
    result.append(r'set(%s_SCRIPTS "%s")' % (package_name,
                                     ';'.join(scripts)))
    allprefix = package_dir.get('', None)

    # Remove packages with '.' separators.  setuptools requires
    # specifying submodules.  The symlink approach of catkin does not
    # work with submodules.  In the common case, this does not matter
    # as the submodule is within the containing module.  We verify
    # this assumption, and if it passes, we remove submodule packages.
    if not allprefix:
        for pkg in pkgs:
            splits = pkg.split('.')
            # hack: ignore write-combining setup.py files for msg and srv
            # files
            if len(splits) > 1 and splits[1] not in ['msg', 'srv']:
                top_level = splits[0]
                top_level_dir = package_dir[top_level]
                expected_dir = os.path.join(top_level_dir,
                                            os.path.join(splits[1:]))
                actual_dir = package_dir[pkg]
                if not samefile(expected_dir, actual_dir):
                    raise RuntimeError(
                        "catkin_export_python does not support setup.py files that combine across multiple directories")

    # If checks pass, remove all submodules
    pkgs = [p for p in pkgs if '.' not in p]

    resolved_pkgs = []
    for pkg in pkgs:
        if allprefix:
            resolved_pkgs += [os.path.join(allprefix, pkg)]
        else:
            resolved_pkgs += [package_dir[pkg]]

    result.append(r'set(%s_PACKAGES "%s")' % (package_name, ';'.join(pkgs)))
    result.append(r'set(%s_PACKAGE_DIRS "%s")' % (package_name, ';'.join(resolved_pkgs).replace("\\", "/")))
    return result


class Dummy:
    '''
    Used as a replacement for modules setuptools and distutils.core,
    defines a setup function.
    '''

    def __init__(self, package_name, outfile):
        self.package_name = package_name
        self.outfile = outfile

    def setup(self, *args, **kwargs):
        '''
        Checks kwargs and writes a scriptfile
        '''
        if 'version' not in kwargs:
            sys.stderr.write("\n*** Unable to find 'version' in setup.py of %s" % self.package_name)
            raise RuntimeError("version not found in setup.py")
        version = kwargs['version']
        if 'package_dir' not in kwargs:
            raise RuntimeError(r'package_dir not in setup.py')
        package_dir = kwargs['package_dir']

        pkgs = kwargs.get('packages', [])
        scripts = kwargs.get('scripts', [])

        result = generate_cmake_file(package_name=self.package_name,
                                     version=version,
                                     scripts=scripts,
                                     package_dir=package_dir,
                                     pkgs=pkgs)
        with open(self.outfile, 'w') as out:
            out.write('\n'.join(result))


def main():
    parser = ArgumentParser(description='Utility to read setup.py values from cmake macros. Creates a file with CMake set commands setting variables.')
    parser.add_argument('package_name', help='Name of catkin package')
    parser.add_argument('setupfile_path', help='Full path to setup.py')
    parser.add_argument('outfile', help='Where to write result to')

    args = parser.parse_args()

    # print("%s" % sys.argv)
    # PACKAGE_NAME = sys.argv[1]
    # OUTFILE = sys.argv[3]
    # print("Interrogating setup.py for package %s into %s " % (PACKAGE_NAME, OUTFILE),
    #      file=sys.stderr)

    dummy = Dummy(package_name = args.package_name,
                  outfile = args.outfile)

    sys.modules['setuptools'] = dummy
    sys.modules['distutils.core'] = dummy

    # find the imports in setup.py relatively to make it work before installing catkin
    sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'python'))

    # print("executing %s" % args.setupfile_path)

    # be sure you're in the directory containing
    # setup.py so the sys.path manipulation works,
    # so the import of __version__ works
    os.chdir(os.path.dirname(os.path.abspath(args.setupfile_path)))

    with open(args.setupfile_path, 'r') as fh:
        exec(fh.read())


if __name__ == '__main__':
    main()
