#!/usr/bin/env python

from __future__ import print_function
import sys
import argparse

try:
    from catkin_pkg.package import parse_package
except ImportError as e:
    sys.exit('ImportError: "from catkin_pkg.package import parse_package" failed: %s\nMake sure that you have installed "catkin_pkg", it is up to date and on the PYTHONPATH.' % e)


def _get_output(package):
    """
    returns a list of strings with cmake commands to execute to set cmake variables

    :param package: Package object
    :returns: list of str, lines to output
    """
    values = {}
    values['VERSION'] = '"%s"' % package.version

    values['MAINTAINER'] = '"%s"' % (', '.join([str(m) for m in package.maintainers]))

    values['BUILD_DEPENDS'] = ' '.join(['"%s"' % str(d) for d in package.build_depends])
    values['RUN_DEPENDS'] = ' '.join(['"%s"' % str(d) for d in package.run_depends])

    deprecated = [e.content for e in package.exports if e.tagname == 'deprecated']
    values['DEPRECATED'] = '"%s"' % ((deprecated[0] if deprecated[0] else 'TRUE') if deprecated else '')

    output = []
    output.append(r'set(_CATKIN_CURRENT_PACKAGE "%s")' % package.name)
    for k, v in values.items():
        output.append('set(%s_%s %s)' % (package.name, k, v))
    return output


def main(argv=sys.argv[1:]):
    """
    Reads given package_xml and writes extracted variables to outfile.
    """
    parser = argparse.ArgumentParser(description="Read package.xml and write extracted variables to stdout")
    parser.add_argument('package_xml')
    parser.add_argument('outfile')
    args = parser.parse_args(argv)
    package = parse_package(args.package_xml)

    lines = _get_output(package)
    with open(args.outfile, 'w') as ofile:
        ofile.write('\n'.join(lines))


if __name__ == '__main__':
    main()
