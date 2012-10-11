#!/usr/bin/env python

from __future__ import print_function
import argparse
import sys
import os

from catkin_pkg.package import parse_package


def parse_args(args=sys.argv[1:]):
    parser = argparse.ArgumentParser(description='Provides the plugin xml exports for the package.xml found in the path to be inserted into the generated manifest.xml.')
    parser.add_argument('package', help='The package to find plugins for')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--exports', dest="exports", action="store_true", help='Generate the exports')
    group.add_argument('--depends', dest="depends", action="store_true", help='Generate the necessary depends')
    args = parser.parse_args(args=args)
    return args


def _get_output(package, exports=True, depends=False):
    output = []
    plugins = []
    for export in package.exports:
        if 'plugin' in export.attributes:
            plugins.append(export)

    for plugin in plugins:
        if exports:
            output.append('    <%s plugin="%s"/>' % (plugin.tagname, plugin.attributes['plugin']))
        elif depends:
            if plugin.tagname != package.name:
                output.append('  <depend package="%s"/>' % plugin.tagname)
    return output


def main():
    args = parse_args()

    try:
        package = parse_package(args.package)
        output = _get_output(package, args.exports, args.depends)
        print('\n'.join(output))
    except Exception as e:
        sys.exit(str(e))


if __name__ == '__main__':
    main()
