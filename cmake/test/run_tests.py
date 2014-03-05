#!/usr/bin/env python

from __future__ import print_function

import argparse
import errno
import os
import sys
import subprocess
from xml.etree.ElementTree import ElementTree, ParseError

from catkin.test_results import read_junit
from catkin.tidy_xml import tidy_xml


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(description='Runs the test command passed as an argument and verifies that the expected result file has been generated.')
    parser.add_argument('results', help='The path to the xunit result file')
    parser.add_argument('command', nargs='+', help='The test command to execute')
    parser.add_argument('--working-dir', nargs='?', help='The working directory for the executed command')
    parser.add_argument('--return-code', action='store_true', default=False, help='Set the return code based on the success of the test command')
    args = parser.parse_args(argv)

    # if result file exists remove it before test execution
    if os.path.exists(args.results):
        os.remove(args.results)
    # if placeholder (indicating previous failure) exists remove it before test execution
    placeholder = os.path.join(os.path.dirname(args.results), 'MISSING-%s' % os.path.basename(args.results))
    if os.path.exists(placeholder):
        os.remove(placeholder)

    work_dir_msg = ' with working directory "%s"' % args.working_dir if args.working_dir is not None else ''
    cmds_msg = ''.join(['\n  %s' % cmd for cmd in args.command])
    print('-- run_tests.py: execute commands%s%s' % (work_dir_msg, cmds_msg))

    rc = 0
    for cmd in args.command:
        rc = subprocess.call(cmd, cwd=args.working_dir, shell=True)
        if rc:
            break

    print('-- run_tests.py: verify result "%s"' % args.results)

    if os.path.exists(args.results):
        # if result file exists ensure that it contains valid xml
        tree = None
        try:
            tree = ElementTree(None, args.results)
        except ParseError:
            #print('Invalid XML in result file "%s"' % args.results)
            tidy_xml(args.results)
            try:
                tree = ElementTree(None, args.results)
            except ParseError as e:
                print('Invalid XML in result file "%s" (even after trying to tidy it): %s ' % (args.results, str(e)), file=sys.stderr)
                rc = 1
        if tree:
            _, num_errors, num_failures = read_junit(args.results)
            if num_errors or num_failures:
                rc = 1
    else:
        rc = 1
        # if result file does not exist create placeholder which indicates failure
        print('Cannot find results, writing failure results to "%s"' % placeholder, file=sys.stderr)
        # create folder if necessary
        if not os.path.exists(os.path.dirname(args.results)):
            try:
                os.makedirs(os.path.dirname(args.results))
            except OSError as e:
                # catch case where folder has been created in the mean time
                if e.errno != errno.EEXIST:
                    raise
        with open(placeholder, 'w') as f:
            data = {'test': os.path.basename(args.results), 'test_file': args.results}
            f.write('''<?xml version="1.0" encoding="UTF-8"?>
<testsuite tests="1" failures="1" time="1" errors="0" name="%(test)s">
  <testcase name="test_ran" status="run" time="1" classname="Results">
    <failure message="Unable to find test results for %(test)s, test did not run.\nExpected results in %(test_file)s" type=""/>
  </testcase>
</testsuite>''' % data)

    if args.return_code:
        return rc
    return 0


if __name__ == '__main__':
    sys.exit(main())
