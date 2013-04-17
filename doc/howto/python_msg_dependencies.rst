.. _python_msg_dependencies:

Python message or service dependencies
--------------------------------------

When your Python package depends on ROS messages or services, they
must be defined by catkin packages like std_msgs_ and sensor_msgs_,
which are used as examples below.

For each Python message dependency, ``package.xml`` must provide a
``<run_depend>`` with the ROS package name::

  <run_depend>std_msgs</run_depend>
  <run_depend>sensor_msgs</run_depend>

If your package contains both Python and :ref:`cpp_msg_dependencies`,
it needs both a ``<build_depend>`` and a ``<run_depend>``.  The C++
references are resolved at compile time and the Python references at
run time.

.. _sensor_msgs: http://www.ros.org/wiki/sensor_msgs
.. _std_msgs: http://www.ros.org/wiki/std_msgs
