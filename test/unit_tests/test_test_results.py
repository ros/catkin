import os
import sys
import unittest
import tempfile
import shutil
from cStringIO import StringIO


try:
    # 'import as' required to protect nosetests
    import catkin.test_results as catkin_test_results
except ImportError as impe:
    raise ImportError(
        'Please adjust your pythonpath before running this test: %s' % str(impe))


class TestResultsTest(unittest.TestCase):

    def test_read_junit(self):
        try:
            rootdir = tempfile.mkdtemp()

            result_file = os.path.join(rootdir, 'test1.xml')
            with open(result_file, 'w') as fhand:
                fhand.write('<testsuites tests="5" failures="3" errors="1" time="35" name="AllTests"></testsuites>')
            (num_tests, num_errors, num_failures) = catkin_test_results.read_junit(result_file)
            self.assertEqual((5, 1, 3), (num_tests, num_errors, num_failures))
        finally:
            shutil.rmtree(rootdir)

    def test_test_results(self):
        try:
            rootdir = tempfile.mkdtemp()

            for filename in ['test1.xml', 'test2.xml', 'foo.bar']:
                result_file = os.path.join(rootdir, filename)
                with open(result_file, 'w') as fhand:
                    fhand.write('<testsuites tests="5" failures="3" errors="1" time="35" name="AllTests"></testsuites>')
            results = catkin_test_results.test_results(rootdir)
            self.assertEqual({'test1.xml': (5, 1, 3), 'test2.xml': (5, 1, 3)}, results)
        finally:
            shutil.rmtree(rootdir)

    def test_print_summary(self):
        results = {'test1.xml': (5, 1, 3), 'test2.xml': (7, 2, 4)}
        try:
            oldstdout = sys.stdout
            sys.stdout = StringIO()
            catkin_test_results.print_summary(results)
            summary = sys.stdout.getvalue()
            self.assertTrue('5 tests, 1 errors, 3 failures' in summary, summary)
            self.assertTrue('7 tests, 2 errors, 4 failures' in summary, summary)
            self.assertTrue('12 tests, 3 errors, 7 failures' in summary, summary)

        finally:
            sys.stdout = oldstdout
