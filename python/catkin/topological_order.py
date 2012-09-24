from __future__ import print_function
import os
import sys

from catkin_pkg.package import parse_package
from catkin_pkg.packages import find_package_paths


class PackageData:

    def __init__(self, path, parse_package_arg=parse_package):
        package = parse_package_arg(path)
        self.name = package.name
        self.path = path
        self.build_depends = set([d.name for d in (package.build_depends + package.buildtool_depends)])
        message_generators = [e.content for e in package.exports if e.tagname == 'message_generator']
        self.message_generator = message_generators[0] if message_generators else None
        self.is_metapackage = 'metapackage' in [e.tagname for e in package.exports]

    def __repr__(self):
        return 'name=%s path=%s build_depends=%s message_generator=%s\n' % (self.name, self.path, self.build_depends, self.message_generator)


def _remove_dependency(packages, name):
    for build_depends in [package_data.build_depends for package_data in packages.values()]:
        build_depends.difference_update([name])


def _sort_packages(packages):
    '''
    First returning packages which have message generators and then the rest based on their build_depends.
    '''

    ordered_packages = []
    while len(packages) > 0:
        # find all packages without build dependencies
        message_generators = []
        non_message_generators = []
        for name, data in packages.items():
            if not data.build_depends:
                if data.message_generator:
                    message_generators.append(name)
                else:
                    non_message_generators.append(name)
        # first choose message generators
        if message_generators:
            names = message_generators
        elif non_message_generators:
            names = non_message_generators
        else:
            # in case of a circular dependency pass the list of remaining packages
            ordered_packages.append([None, ', '.join(sorted(packages.keys()))])
            break

        # alphabetic order only for convenience
        names.sort()
        #print('names = %s' % names, file=sys.stderr)

        # add first candidates to ordered list
        # do not add all candidates since removing the depends from the first might affect the next candidates
        name = names[0]
        ordered_packages.append([name, packages[name]])
        # remove package from further processing
        del packages[name]
        _remove_dependency(packages, name)
    return ordered_packages


def topological_order(source_root_dir, whitelisted=None, blacklisted=None):
    paths = find_package_paths(source_root_dir)
    #print('paths = %s' % paths, file=sys.stderr)

    # fetch all meta data
    prefix = os.path.abspath(source_root_dir) + os.sep
    package_data_list = []
    for path in paths:
        data = PackageData(os.path.join(source_root_dir, path))
        # make path relative to root dir
        if data.path.startswith(prefix):
            data.path = data.path[len(prefix):]
        package_data_list.append(data)
    return _topological_order_packages(package_data_list, whitelisted, blacklisted)


def _topological_order_packages(package_data_list, whitelisted=None, blacklisted=None):
    packages = {}
    for data in package_data_list:
        # skip non-whitelisted packages
        if whitelisted and data.name not in whitelisted:
            continue
        # skip blacklisted packages
        if blacklisted and data.name in blacklisted:
            continue
        if data.name in packages:
            print('Two package with the same name "%s" in the workspace:\n- %s\n- %s' % (data.name, packages[data.name].path, data.path), file=sys.stderr)
            sys.exit(1)
        packages[data.name] = data

    # remove catkin from list of packages
    if 'catkin' in packages:
        del packages['catkin']

    # remove external dependencies from the list
    if packages:
        all_build_depends = reduce(set.union, [p.build_depends for p in packages.values()])
        external_depends = all_build_depends - set(packages.keys())
        #print('external_depends = %s' % external_depends, file=sys.stderr)
        for data in packages.values():
            data.build_depends = set(data.build_depends) - set(external_depends)

    return _sort_packages(packages)


def get_message_generators(ordered_packages):
    return [name for (name, data) in ordered_packages if data.message_generator]
