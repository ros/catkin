#!/usr/bin/env python

from __future__ import print_function
import os
import sys

from argparse import ArgumentParser


def _get_locations(pkgs, package_dir):
    """
    based on setuptools logic and the package_dir dict, builds a dict
    of location roots for each pkg in pkgs.
    See http://docs.python.org/distutils/setupscript.html

    :returns: a dict {pkgname: root} for each pkgname in pkgs (and each of their parents)
    """
    # package_dir contains a dict {package_name: relativepath}
    # Example {'': 'src', 'foo': 'lib', 'bar': 'lib2'}
    #
    # '' means where to look for any package unless a parent package
    # is listed so package bar.pot is expected at lib2/bar/pot,
    # whereas package sup.dee is expected at src/sup/dee
    #
    # if package_dir does not state anything about a package,
    # setuptool expects the package folder to be in the root of the
    # project
    locations = {}
    allprefix = package_dir.get('', '')
    for pkg in pkgs:
        parent_location = None
        splits = pkg.split('.')
        # we iterate over compound name from parent to child
        # so once we found parent, children just append to their parent
        for key_len in range(len(splits)):
            key = '.'.join(splits[:key_len + 1])
            if key not in locations:
                if key in package_dir:
                    locations[key] = package_dir[key]
                elif parent_location is not None:
                    locations[key] = parent_location
                else:
                    locations[key] = allprefix
            parent_location = locations[key]
    return locations


def generate_cmake_file(package_name, version, scripts, package_dir, pkgs):
    """
    Generates lines to add to a cmake file which will set variables

    :param version: str, format 'int.int.int'
    :param scripts: [list of str]: relative paths to scripts
    :param package_dir: {modulename: path}
    :pkgs: [list of str] python_packages declared in catkin package
    """
    prefix = '%s_SETUP_PY' % package_name
    result = []
    result.append(r'set(%s_VERSION "%s")' % (prefix, version))
    result.append(r'set(%s_SCRIPTS "%s")' % (prefix, ';'.join(scripts)))

    # Remove packages with '.' separators.
    #
    # setuptools allows specifying submodules in other folders than
    # their parent
    #
    # The symlink approach of catkin does not work with such submodules.
    # In the common case, this does not matter as the submodule is
    # within the containing module.  We verify this assumption, and if
    # it passes, we remove submodule packages.
    locations = _get_locations(pkgs, package_dir)
    for pkgname, location in locations.items():
        if not '.' in pkgname:
            continue
        splits = pkgname.split('.')
        # hack: ignore write-combining setup.py files for msg and srv files
        if splits[1] in ['msg', 'srv']:
            continue
        # check every child has the same root folder as its parent
        parent_name = '.'.join(splits[:1])
        if location != locations[parent_name]:
            raise RuntimeError(
                "catkin_export_python does not support setup.py files that combine across multiple directories: %s in %s, %s in %s" % (pkgname, location, parent_name, locations[parent_name]))

    # If checks pass, remove all submodules
    pkgs = [p for p in pkgs if '.' not in p]

    resolved_pkgs = []
    for pkg in pkgs:
        resolved_pkgs += [os.path.join(locations[pkg], pkg)]

    result.append(r'set(%s_PACKAGES "%s")' % (prefix, ';'.join(pkgs)))
    result.append(r'set(%s_PACKAGE_DIRS "%s")' % (prefix, ';'.join(resolved_pkgs).replace("\\", "/")))
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
        package_dir = kwargs['package_dir'] if 'package_dir' in kwargs else {}

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
    """
    Script main, parses arguments and invokes Dummy.setup indirectly.
    """
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

    dummy = Dummy(package_name=args.package_name,
                  outfile=args.outfile)

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
