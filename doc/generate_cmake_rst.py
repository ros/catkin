#!/usr/bin/env python

from __future__ import print_function
import argparse
import os
import re
import sys

'''Simple superficial API doc generator for .cmake files'''


def crawl_for_cmake(path):
    '''
    Crawls over path, looking for files named *.cmake,
    returns tuple of full and relative path.
    '''
    cmake_files = []
    for (parentdir, _, files) in os.walk(path):
        for filename in files:
            if not filename.endswith('.cmake'):
                continue
            fullpath = os.path.join(parentdir, filename)
            relpath = os.path.relpath(fullpath, path)
            cmake_files.append((fullpath, relpath))
    return cmake_files


def generate_rst(files):
    '''
    Each of the CMake files is traversed line by line, looking for
    lines like function(...) or macro(...).  For each of these,
    multiple lines of reStructured text are added documenting the
    function.
    '''
    documented = {}
    undocumented = {}
    for (fullpath, relpath) in files:
        last_block = []
        with open(fullpath, 'r') as f:
            lines = f.readlines()
        for line in lines:
            if line.startswith('#'):
                # careful to not strip '\n' in empty comment lines
                last_block.append(line.lstrip('#').rstrip('\n'))
            else:
                declaration = re.match('[a-zA-Z]+\([a-zA-Z_ ]+\)', line)
                if declaration is None:
                    last_block = []
                else:
                    tokens = line.split('(')
                    dec_type = tokens[0].strip()
                    dec_args = tokens[1].strip().rstrip(')').split(' ')

                    if dec_type == 'function' or dec_type == 'macro':
                        rst = []
                        # directives defined in catkin-sphinx
                        dec_line = '.. cmake:macro:: %s(%s)' % (dec_args[0], ', '.join(dec_args[1:]))
                        rst.append(dec_line)
                        rst.append('')
                        rst.append(' defined in %s' % relpath)
                        rst.append('')
                        rst.extend(last_block)

                        if dec_args[0] in documented or dec_args[0] in undocumented:
                            raise RuntimeError('Function/macro with same name "%s" exists multiple times' % dec_args[0])
                        if last_block != []:
                            documented[dec_args[0]] = rst
                        else:
                            undocumented[dec_args[0]] = rst

                    last_block = []

    rst = ['Extracted CMake documentation',
           '=============================']
    for name in sorted(documented.keys()):
        rst.append('')
        rst.extend(documented[name])

    rst.append('')
    rst.append('Not documented CMake functions / macros')
    rst.append('---------------------------------------')
    for name in sorted(undocumented.keys()):
        rst.append('')
        rst.extend(undocumented[name])

    return rst


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Crawls a path for .cmake files and extract documentation of functions and macros into reStructured text.')
    parser.add_argument('path', nargs='?', default='.', help='The path to be crawled')
    parser.add_argument('-o', '--output', help='The name of the generated rst file')
    args = parser.parse_args()

    cmake_files = crawl_for_cmake(args.path)
    lines = generate_rst(cmake_files)
    if args.output:
        with open(args.output, 'w') as f:
            f.write('\n'.join(lines))
    else:
        for line in lines:
            print(line)
    sys.exit(0)
