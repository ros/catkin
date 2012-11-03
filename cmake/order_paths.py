#!/usr/bin/env python

from __future__ import print_function

from argparse import ArgumentParser
import os


def main():
    """
    Order a list of paths according to a list of prefixes which define the order.
    """
    parser = ArgumentParser(description='Utility to order a list of paths according to a list of prefixes. Creates a file with CMake set command setting a variable.')
    parser.add_argument('outfile', help='The filename of the generated CMake file')
    parser.add_argument('--paths-to-order', nargs='*', help='The semicolon-separated paths to order')
    parser.add_argument('--prefixes', nargs='*', help='The semicolon-separated prefixes defining the order')
    args = parser.parse_args()

    ordered_paths = order_paths(args.paths_to_order, args.prefixes)

    # create directory if necessary
    outdir = os.path.dirname(args.outfile)
    if not os.path.exists(outdir):
        os.makedirs(outdir)

    with open(args.outfile, 'w') as fh:
        fh.write('set(ORDERED_PATHS "%s")' % ';'.join(ordered_paths))


def order_paths(paths_to_order, prefixes):
    # the ordered paths contains a list for each prefix plus one more which contains paths which do not match one of the prefixes
    ordered_paths = [[] for _ in range(len(prefixes) + 1)]

    for path in paths_to_order:
        # put each include directory into the slot where it matches the prefix, or last otherwise
        index = 0
        for prefix in prefixes:
            if path == prefix or path.startswith(prefix + os.sep) or (os.altsep and path.startswith(prefix + os.altsep)):
                break
            index += 1
        ordered_paths[index].append(path)

    # flatten list of lists
    return [j for i in ordered_paths for j in i]


if __name__ == '__main__':
    main()
