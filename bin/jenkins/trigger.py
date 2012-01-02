#!/usr/bin/env python
import argparse
import subprocess
import os
import pprint
from construct_graph import topological_sort, buildable_graph_from_dscs
def parse_options():
    parser = argparse.ArgumentParser(
             description='Queue all builds in dependency order.')
    parser.add_argument(dest='dscs',
           help='A list of dsc files.')
    parser.add_argument('--commit', dest='commit',
           help='Really?', action='store_true')
    return parser.parse_args()

def deb_list(dscs):
    dsc_list = []
    for subdir, _, files in os.walk(dscs):
        dsc_list += [os.path.join(subdir, f) for f in files if os.path.splitext(f)[1] in '.dsc']
    graph = buildable_graph_from_dscs(dsc_list)
    pprint.pprint(graph)
    jobs = topological_sort(graph)
    pprint.pprint(jobs)

if __name__ == "__main__":
    args = parse_options()
    deb_list(args.dscs)
