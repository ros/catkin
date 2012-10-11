#!/usr/bin/env python




from __future__ import print_function
import argparse
import sys
import os

from catkin_pkg.package import parse_package


def parse_args(args=sys.argv[1:]):
    parser = argparse.ArgumentParser(description='Provides the plugin xml exports for the package.xml found in the path to be inserted into the generated manifest.xml.')
    parser.add_argument('file', help='The project to find plugins for')
    parser.add_argument('--exports', dest="exports", action="store_true", help='Generate the exports')
    parser.add_argument('--depends', dest="depends", action="store_true", help='Generate the necessary depends')
    args = parser.parse_args(args=args)
    if not (args.exports or args.depends ):
        parser.error("exports or depends options required")
    return args





def main():
    args = parse_args()

    plugins = []
    try:
        package = parse_package(args.file)
        for e in package.exports:
            if 'plugin' in e.attributes:
                plugins.append(e)

        for p in plugins:
            if args.exports:
                print('<%s plugin="%s"/>' % (p.tagname, p.attributes['plugin']) )
            elif args.depends:
                if p.tagname != package.name:
                    print('<depend package="%s"/>' % p.tagname)


    except Exception as e:
        sys.exit(str(e))


if __name__ == '__main__':
    main()
