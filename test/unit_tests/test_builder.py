# -*- coding: utf-8 -*-

import os
import unittest
# import tempfile
# import shutil
# import mock

try:
    import catkin.builder
    # from catkin.builder import build_workspace_in_isolation
except ImportError as e:
    raise ImportError(
        'Please adjust your pythonpath before running this test: %s' % str(e)
    )


class BuilderTest(unittest.TestCase):
    # TODO: Add tests for catkin_make and catkin_make_isolated

    def test_run_command_unicode(self):
        backup_Popen = catkin.builder.subprocess.Popen

        class StdOut(object):
            def __init__(self, popen):
                self.__popen = popen

            def readline(self):
                self.__popen.returncode = 0
                return u'\u2018'

        class MockPopen(object):
            def __init__(self, *args, **kwargs):
                self.returncode = None
                self.stdout = StdOut(self)

            def wait(self):
                return True

        try:
            catkin.builder.subprocess.Popen = MockPopen
            catkin.builder.run_command(['false'], os.getcwd(), True, True)
        finally:
            catkin.builder.subprocess.Popen = backup_Popen
