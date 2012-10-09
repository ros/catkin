Subfolders of catkin/test:
==========================

Use python-nose to run tests, get it from apt-get.
Run tests from the catkin folder using just::

  $ nosetests
  $ nosetests test/path/to/python_file

Is it also possible to run tests from the test folder.

To run a single test function, run e.g.::

  $ nosetests path/to/python_file.py:ClassName.testmethod


src
---

a test workspace with mock catkin stacks

mock_resources
--------------

mock catkin stacks to run tests against

local_tests
-----------

tests that require no network connection (quick)

network_tests
-------------

tests that checkout all core ros stacks, and require network connection (slow)

tmp
---

created by network tests, downloaded ros core packages to run make against.
Placed here to avoid long test durations due to duplicate downloads.


checks
------
scripts to run manually that ensure catkin is viable for ROS distros.
Those scripts do many dangerous things, such as sudo rm -rf /opt/ros.
