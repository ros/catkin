#!/usr/bin/env python

import yaml
import argparse
import sys

def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Parse a rosinstall file, and output fields in a CSV format.")
    parser.add_argument('ri_file',help='The rosinstall file to parse.')
    parser.add_argument('vcs_type',help='The vcs type')
    parser.add_argument('FIELD',nargs='*',help='The field to extract from the ros install file per entry.')
    return parser.parse_args(args=argv)

args = parse_args()
ri_file = yaml.load(open(args.ri_file))
for info in [ x[args.vcs_type] for x in ri_file if args.vcs_type in x] :
    infos = []
    for x in args.FIELD:
        val = info.get(x,None)
        if val:
            infos.append(val)
    if infos:
        sys.stdout.write(','.join(infos)+'\n')
