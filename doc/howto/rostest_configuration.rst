.. _rostest_configuration:

Configuring rostest
-------------------

Rostest_ is needed whenever unit tests require a roscore_ for running
ROS nodes.


package.xml
:::::::::::


The ``rostest`` package is needed at build time, because it defines
the ``add_rostest()`` CMake command::

  <build_depend>rostest</build_depend>

Do not declare a ``<test_depend>``, because it will conflict with the
required ``<build_depend>``.


CMakeLists.txt
::::::::::::::

You must also include ``rostest`` in your ``find_package()``
components::

  find_package(catkin REQUIRED COMPONENTS rostest ...)

Finally, declare your rostest launch scripts::

  add_rostest(tests/your_first_rostest.test)
  add_rostest(tests/your_second_rostest.test)

If your rostest should contain a gtest executable you can use the
following convenient function::

  add_rostest_gtest(your_third_rostest_target tests/your_third_rostest.test src/test/your_third_rostest.cpp [more cpp files])
  target_link_libraries(your_third_rostest_target [libraries to depend on, e.g. ${catkin_LIBRARIES}])

For more information how to write and run rostests please go to the
Rostest_ wiki page.

.. _roscore: http://www.ros.org/wiki/roscore
.. _Rostest: http://ros.org/wiki/rostest
