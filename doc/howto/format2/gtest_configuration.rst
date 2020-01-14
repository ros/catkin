.. _gtest_configuration_2:

Configuring gtest for C++
-------------------------

Gtest_ is the Google framework for running C++ unit tests.  It is a
pure C++ framework.  No ROS environment is available.  See:
:ref:`rostest_configuration` if your tests need a running roscore_.


CMakeLists.txt
::::::::::::::

Declare each gtest like this::

  if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_your_node tests/test_your_node.cpp)
    target_link_libraries(test_your_node ${catkin_LIBRARIES})
  endif()

This example assumes your tests are defined in the ``tests/``
subdirectory in your source tree.

If other libraries are needed to compile your test program, see
:ref:`building_executables_2` for details.

.. _Gtest: http://wiki.ros.org/gtest
.. _roscore: http://wiki.ros.org/roscore
