#!/usr/bin/env python

from __future__ import print_function
import sys
import argparse

from catkin_pkg.package import parse_package


def main():
    """
    Reads package.xml and writes extracted variables to stdout.
    """
    parser = argparse.ArgumentParser(description="Read package.html and write extracted variables to stdout")
    parser.add_argument('package_xml')
    parser.add_argument('outfile')
    args = parser.parse_args()
    package = parse_package(args.package_xml)

    values = {}
    values['VERSION'] = '"%s"' % package.version

    values['MAINTAINER'] = '"%s"' % (', '.join([str(m) for m in package.maintainers]))

    values['BUILD_DEPENDS'] = ' '.join(['"%s"' % str(d) for d in package.build_depends])
    values['RUN_DEPENDS'] = ' '.join(['"%s"' % str(d) for d in package.run_depends])

    with open(args.outfile, 'w') as ofile:
        print(r'set(_CATKIN_CURRENT_PACKAGE "%s")' % package.name, file=ofile)
        for k, v in values.items():
            print('set(%s_%s %s)' % (package.name, k, v), file=ofile)


if __name__ == '__main__':
    main()
