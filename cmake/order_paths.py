from __future__ import print_function

import os
import sys
from argparse import ArgumentParser

try:
    from catkin_pkg.workspaces import get_spaces
except ImportError as e:
    sys.exit('ImportError: "catkin_pkg.workspaces import get_spaces" failed: %s\nMake sure that you have installed "catkin_pkg", it is up to date and on the PYTHONPATH.' % e)
try:
    from catkin_pkg.workspaces import order_paths
except ImportError as e:
    sys.exit('ImportError: "from catkin_pkg.workspaces import order_paths" failed: %s\nMake sure that you have installed "catkin_pkg", it is up to date and on the PYTHONPATH.' % e)


def main():
    """Order a list of paths according to a list of prefixes which define the order."""
    parser = ArgumentParser(description='Utility to order a list of paths according to a list of prefixes. Creates a file with CMake set command setting a variable.')
    parser.add_argument('outfile', help='The filename of the generated CMake file')
    parser.add_argument('--paths-to-order', nargs='*', help='The semicolon-separated paths to order')
    parser.add_argument('--prefixes', nargs='*', help='The semicolon-separated prefixes defining the order')
    args = parser.parse_args()

    # resolve the source space if any
    spaces = []
    for prefix in args.prefixes:
        spaces.append(prefix)
        spaces += get_spaces([prefix])

    ordered_paths = order_paths(args.paths_to_order, spaces)

    # create directory if necessary
    outdir = os.path.dirname(args.outfile)
    if not os.path.exists(outdir):
        os.makedirs(outdir)

    with open(args.outfile, 'w') as fh:
        fh.write('set(ORDERED_PATHS "%s")' % ';'.join(ordered_paths))


if __name__ == '__main__':
    main()
