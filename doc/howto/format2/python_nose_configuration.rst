.. _python_nose_configuration_2:

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
    catkin_add_nosetests(tests/test_your_node.py)
  endif()

For more info, please have a look at the :ref:`API <catkin_add_nosetests_ref>`.

.. _Nosetest: http://wiki.ros.org/nosetest
.. _roscore: http://wiki.ros.org/roscore
.. _unittest: http://wiki.ros.org/unittest
