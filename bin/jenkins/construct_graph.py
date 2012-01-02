#!/usr/bin/env python

import yaml
import sys
import pprint
import re
import os

def parse_dsc(package_name, dsc_file):
    dsc = yaml.load(open(dsc_file))
    return {package_name:set([dep.strip() for dep in dsc['Build-Depends'].split(',')])}

def split_dsc(dsc):
    dsc_base = os.path.basename(dsc)
    m = re.search('(.+)_([\d\.]+)-(.+)\.dsc', dsc_base)
    package_name = m.group(1)
    version = tuple([int(x) for x in m.group(2).split('.')])
    debian_version = m.group(3)
    return (package_name, version, debian_version, dsc)

def sort_dscs(dscs):
    dscs_parsed = [ split_dsc(x) for x in dscs]
    dscs_parsed = sorted(dscs_parsed)
    dscs_parsed.reverse()
    return dscs_parsed

def grab_latests_dscs(dscs):
    dscs = sort_dscs(dscs)
    latest_set = set()
    for package_name, _version, _debian_version, dsc in dscs:
        if not package_name in latest_set:
            latest_set.add((package_name, dsc))
    return latest_set


def buildable_graph_from_dscs(dscs):
    dscs = grab_latests_dscs(dscs)
    graph = {}
    for package_name, dsc in dscs:
        graph.update(parse_dsc(package_name, dsc))

    ourpackages = [package for package in graph.keys()]
    graph_we_can_build = {}

    for key, deps in graph.iteritems():
        graph_we_can_build[key] = deps.intersection(ourpackages)
    return graph_we_can_build


def topological_sort(graph):
    '''
    http://en.wikipedia.org/wiki/Topological_sorting
    L <- Empty list that will contain the sorted elements
    S <- Set of all nodes with no incoming edges
    while S is non-empty do
        remove a node n from S
        insert n into L
        for each node m with an edge e from n to m do
            remove edge e from the graph
            if m has no other incoming edges then
                insert m into S
    if graph has edges then
        return error (graph has at least one cycle)
    else 
        return L (a topologically sorted order)
    '''
    L = []
    S = [package for package in graph.keys() if len(graph[package]) == 0]
    while S:
        n = S.pop()
        L.append(n)
        for m in [m for m in graph.keys() if n in graph[m]]:
            graph[m].remove(n)
            if len(graph[m]) == 0:
                S.append(m)
    return L


if __name__ == "__main__":
    graph = buildable_graph_from_dscs(sys.argv[1:])
    pprint.pprint(graph)
    jobs = topological_sort(graph)
    pprint.pprint(jobs)
