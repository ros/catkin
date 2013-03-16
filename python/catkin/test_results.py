# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function
import os
from xml.etree.ElementTree import ElementTree


def read_junit(filename):
    """
    parses xml file expected to follow junit/gtest conventions
    see http://code.google.com/p/googletest/wiki/AdvancedGuide#Generating_an_XML_Report

    :param filename: str junit xml file name
    :returns: num_tests, num_errors, num_failures
    :raises ParseError: if xml is not well-formed
    :raises IOError: if filename does not exist
    """
    tree = ElementTree()
    root = tree.parse(filename)
    num_tests = int(root.attrib['tests'])
    num_errors = int(root.attrib['errors'])
    num_failures = int(root.attrib['failures'])
    return (num_tests, num_errors, num_failures)


def test_results(test_results_dir):
    '''
    Collects test results by parsing all xml files in given path,
    attempting to interpret them as junit results.

    :param test_results_dir: str foldername
    :returns: dict {rel_path, (num_tests, num_errors, num_failures)}
    '''
    results = {}
    for dirpath, dirnames, filenames in os.walk(test_results_dir):
        # do not recurse into folders starting with a dot
        dirnames[:] = [d for d in dirnames if not d.startswith('.')]
        for filename in [f for f in filenames if f.endswith('.xml')]:
            filename_abs = os.path.join(dirpath, filename)
            name = filename_abs[len(test_results_dir) + 1:]
            try:
                num_tests, num_errors, num_failures = read_junit(filename_abs)
            except Exception as e:
                print('Skipping "%s": %s' % (name, str(e)))
                continue
            results[name] = (num_tests, num_errors, num_failures)
    return results


def aggregate_results(results, callback_per_result=None):
    """
    Aggregate results

    :param results: dict as from test_results()
    :returns: tuple (num_tests, num_errors, num_failures)
    """
    sum_tests = sum_errors = sum_failures = 0
    for name in sorted(results.keys()):
        (num_tests, num_errors, num_failures) = results[name]
        sum_tests += num_tests
        sum_errors += num_errors
        sum_failures += num_failures
        if callback_per_result:
            callback_per_result(name, num_tests, num_errors, num_failures)
    return sum_tests, sum_errors, sum_failures


def print_summary(results, show_stable=False, show_unstable=True):
    """
    print summary to stdout

    :param results: dict as from test_results()
    :param show_stable: print tests without failures extra
    :param show_stable: print tests with failures extra
    """
    def callback(name, num_tests, num_errors, num_failures):
        if show_stable and not num_errors and not num_failures:
            print('%s: %d tests' % (name, num_tests))
        if show_unstable and (num_errors or num_failures):
            print('%s: %d tests, %d errors, %d failures' % (name, num_tests, num_errors, num_failures))
    sum_tests, sum_errors, sum_failures = aggregate_results(results, callback)
    print('Summary: %d tests, %d errors, %d failures' % (sum_tests, sum_errors, sum_failures))
