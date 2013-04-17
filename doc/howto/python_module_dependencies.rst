.. _python_module_dependencies:

Python module dependencies
--------------------------

When your Python package imports other python modules, ``package.xml``
should provide a ``<run_depend>`` with the appropriate package name.

For system dependencies, like ``python-numpy`` or ``python-yaml``, use
the corresponding rosdep name::

  <run_depend>python-numpy</run_depend>
  <run_depend>python-yaml</run_depend>

These names are usually already defined in the `rosdistro
repository`_.  If you need a module not yet defined there, please
`fork that repository and add them`_.

Several ROS infrastructure modules, like ``python-rospkg`` or
``python-rosdep`` itself, apply to multiple ROS releases and are
released independently of them.  Resolve those module dependencies
like other system packages, using the rosdep name::

  <run_depend>python-rosdep</run_depend>
  <run_depend>python-rospkg</run_depend>

When you import from another ROS catkin package, like ``rospy`` or
``std_msgs``, always use the catkin package name::

  <run_depend>rospy</run_depend>
  <run_depend>std_msgs</run_depend>

Your ``CMakeLists.txt`` need not specify Python-only dependencies.
They are resolved automatically via ``sys.path``.

.. _`fork that repository and add them`: http://ros.org/doc/independent/api/rosdep/html/contributing_rules.html
.. _`rosdistro repository`: https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml
