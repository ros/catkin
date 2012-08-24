#!/usr/bin/env python

from __future__ import print_function
import argparse
import os
import subprocess
from xml.etree.ElementTree import ElementTree

print('run_tests:', os.environ['ROS_PACKAGE_PATH'])

parser = argparse.ArgumentParser(description='Runs the test command passed as an argument and verifies that the expected result file has been generated.')
parser.add_argument('command', nargs='+', help='The test command to execute')
parser.add_argument('--results', help='The path to the xunit result file')
parser.add_argument('--working-dir', nargs='?', help='The working directory for the executed command')
args = parser.parse_args()

print('-- run_tests.py: execute commands%s%s' % (' with working directory "%s"' % args.working_dir if args.working_dir is not None else '', (''.join(['\n  %s' % cmd for cmd in args.command]))))
for cmd in args.command:
    subprocess.call(cmd, cwd=args.working_dir, shell=True)

print('-- run_tests.py: verify result "%s"' % args.results)

if os.path.exists(args.results):
    # if result file exists ensure that it contains valid xml

    try:
        root = ElementTree(None, args.results)
        print(root)
    except Exception as e:
        print('Invalid XML in result file "%s"' % args.results)
else:
    # if result file does not exist create placeholder which indicates failure
    placeholder = os.path.join(os.path.dirname(args.results), 'MISSING-%s' % os.path.basename(args.results))
    print('Cannot find results, writing failure results to "%s"' % placeholder)
    # create folder if necessary
    if not os.path.exists(os.path.dirname(args.results)):
        os.makedirs(os.path.dirname(args.results))
    with open(placeholder, 'w') as f:
        data = {'test': os.path.basename(args.results), 'test_file': args.results}
        f.write('''<?xml version="1.0" encoding="UTF-8"?>
<testsuite tests="1" failures="1" time="1" errors="0" name="%(test)s">
  <testcase name="test_ran" status="run" time="1" classname="Results">
    <failure message="Unable to find test results for %(test)s, test did not run.\nExpected results in %(test_file)s" type=""/>
  </testcase>
</testsuite>''' % data)
