#!/usr/bin/env python

from __future__ import print_function
import em
import os
from xml.sax.saxutils import escape
import argparse
import jenkins
import pprint
from construct_graph import topological_sort, buildable_graph_from_dscs

def parse_options():
    parser = argparse.ArgumentParser(
             description='Create a set of jenkins jobs '
             'for source debs and binary debs for a catkin package.')
    parser.add_argument('--fqdn', dest='fqdn',
           help='The source repo to push to, fully qualified something...',
           default='50.28.27.175')
    parser.add_argument(dest='rosdistro',
           help='The ros distro. electric, fuerte, galapagos')
    parser.add_argument('--distros', nargs='+',
           help='A list of debian distros. Default: %(default)s',
           default=['lucid', 'natty', 'oneiric'])
    parser.add_argument('--commit', dest='commit',
           help='Really?', action='store_true')
    parser.add_argument(dest='release_uri',
           help='A release repo uri..')
    parser.add_argument('--dscs', dest='dscs', help='A directory with all the dscs that jenkins builds.')
    return parser.parse_args()

class Templates(object):
    template_dir = os.path.dirname(__file__)
    config_sourcedeb = os.path.join(template_dir, 'config.source.xml.em') #A config.xml template for sourcedebs.
    command_sourcedeb = os.path.join(template_dir, 'source_build.sh.em') #The bash script that the sourcedebs config.xml runs.
    config_bash = os.path.join(template_dir, 'config.bash.xml.em') #A config.xml template for something that runs a shell script
    command_binarydeb = os.path.join(template_dir, 'binary_build.sh.em') #builds binary debs.
    config_binarydeb = os.path.join(template_dir, 'config.binary.xml.em') #A config.xml template for something that runs a shell script

def expand(config_template, d):
    with open(config_template) as fh:
        file_em = fh.read()
    s = em.expand(file_em, **d)
    return s

def create_jenkins(jobname, config):
    j = jenkins.Jenkins('http://hudson.willowgarage.com:8080', 'username', 'password')
    jobs = j.get_jobs()
    print("working on job", jobname)
    if jobname in [job['name'] for job in jobs]:
        j.reconfig_job(jobname, config)
    else:
        j.create_job(jobname, config)

def sourcedeb_job_name(d):
    return "%(ROS_DISTRO)s_%(PACKAGE)s_sourcedeb" % d

def create_sourcedeb_config(d):
    #Create the bash script the runs inside the job
    #need the command to be safe for xml.
    d['COMMAND'] = escape(expand(Templates.command_sourcedeb, d))
    return expand(Templates.config_sourcedeb, d)

def create_binarydeb_config(d):
    d['COMMAND'] = escape(expand(Templates.command_binarydeb, d))
    return expand(Templates.config_binarydeb, d)

def binary_begin_end(d):
    beginning = "ros-%(ROS_DISTRO)s-" % d
    end = "_binarydeb_%(DISTRO)s_%(ARCH)s" % d
    return (beginning, end)

def binarydeb_job_name(d):
    beginning, end = binary_begin_end(d)
    return beginning + d['PACKAGE'].replace('_', '-') + end

def calc_child_jobs(d, jobgraph):
    children = []
    beginning, end = binary_begin_end(d)
    if jobgraph:
        pname = beginning + d['PACKAGE'].replace('_', '-')
        for package,deps in jobgraph.iteritems():
            if pname in deps:
                children.append(package + end)
    d["CHILD_PROJECTS"] = children

def binarydeb_jobs(package, rosdistro, distros, fqdn, jobgraph, ros_package_repo="http://50.28.27.175/repos/building"):
    d = dict(
        ROS_DISTRO=rosdistro,
        DISTROS=distros,
        FQDN=fqdn,
        ROS_PACKAGE_REPO=ros_package_repo,
        PACKAGE=package
    )
    jobs = []
    for distro in distros:
        for arch in ('i386', 'amd64'):
            d['ARCH'] = arch
            d['DISTRO'] = distro
            calc_child_jobs(d, jobgraph)
            config = create_binarydeb_config(d)
            print(config)
            job_name = binarydeb_job_name(d)
            jobs.append((job_name, config))
    return jobs

def sourcedeb_job(package, rosdistro, distros, fqdn, release_uri, child_projects):
    d = dict(
    RELEASE_URI=release_uri,
    ROS_DISTRO=rosdistro,
    FQDN=fqdn,
    DISTROS=distros,
    CHILD_PROJECTS=child_projects,
    PACKAGE=package
    )
    return  (sourcedeb_job_name(d), create_sourcedeb_config(d))

def deb_job_graph(dscs):
    dsc_list = []
    for subdir, _, files in os.walk(dscs):
        dsc_list += [os.path.join(subdir, f) for f in files if os.path.splitext(f)[1] in '.dsc']
    graph = buildable_graph_from_dscs(dsc_list)
    return graph

def doit():
    args = parse_options()
    job_graph = None
    if args.dscs:
        job_graph = deb_job_graph(args.dscs)

    package = os.path.splitext(os.path.basename(args.release_uri))[0]

    binary_jobs = binarydeb_jobs(package, args.rosdistro, args.distros, args.fqdn, job_graph)
    child_projects = zip(*binary_jobs)[0] #unzip the binary_jobs tuple.
    source_job = sourcedeb_job(package, args.rosdistro, args.distros, args.fqdn, args.release_uri, child_projects)
    jobs = [source_job] + binary_jobs
    for job_name, config in jobs:
        if args.commit:
            create_jenkins(job_name, config)

    print("="*80)
    print ("Summary: %d jobs configured.  Listed below." % len(jobs))
    for job_name, config in jobs:
        print ("  %s" % job_name)
    if not args.commit:
        print("This was not pushed to the server.  If you want to do so use ",
              "--commit to do it for real.")
    print("="*80)

if __name__ == "__main__":
    doit()
