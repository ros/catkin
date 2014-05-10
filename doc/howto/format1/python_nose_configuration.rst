.. _python_nose_configuration:

Configuring Python nose tests
-----------------------------

Nosetest_ is the framework for running Python unit tests.  No ROS
environment is available.  See: :ref:`rostest_configuration` if your
tests need a running roscore_.


package.xml
:::::::::::

The unittest_ package is needed for testing::

  <test_depend>unittest</test_depend>


CMakeLists.txt
::::::::::::::

Declare each nose test like this::

  if (CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(test_your_node tests/test_your_node.py)
  endif()

This example assumes your tests are defined in the ``tests/``
subdirectory in your source tree.

.. _Nosetest: http://www.ros.org/wiki/nosetest
.. _roscore: http://www.ros.org/wiki/roscore
.. _unittest: http://www.ros.org/wiki/unittest
