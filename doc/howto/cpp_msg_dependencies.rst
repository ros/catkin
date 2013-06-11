.. _cpp_msg_dependencies:

C++ message or service dependencies
-----------------------------------

When your C++ programs depend on ROS messages or services, they must
be defined by catkin packages like std_msgs_ and sensor_msgs_, which
are used in the examples below.

There are several kinds of dependencies which must be declared in your
``package.xml`` and ``CMakeLists.txt`` files to resolve message
references.


package.xml
:::::::::::

For each C++ message dependency, ``package.xml`` should provide a
``<depend>`` with the ROS package name::

  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>

The ``<depend>`` element is equivalent to specifying both
``<build_depend>`` and ``<run_depend>``, which are both recommended
for message dependencies.

CMakeLists.txt
::::::::::::::

For C++ access to ROS messages, CMake needs to find the message or
service headers::

  find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs)
  include_directories(include ${catkin_INCLUDE_DIRS})

The ``include`` parameter is needed only if that subdirectory of your
package contains headers also needed to compile your programs.

Since you presumably have build targets using the message or service
headers, add this to ensure all their headers get built before any
targets that need them::

  add_dependencies(your_program ${catkin_EXPORTED_TARGETS})
  add_dependencies(your_library ${catkin_EXPORTED_TARGETS})

.. _sensor_msgs: http://www.ros.org/wiki/sensor_msgs
.. _std_msgs: http://www.ros.org/wiki/std_msgs
