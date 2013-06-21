.. _how_to_do_common_tasks_1:

How to do common tasks (format 1)
=================================

.. warning::

   These instructions are for the original `<package format="1">`.

   For the most recent package format, see: :ref:`how_to_do_common_tasks`.

When writing a new ROS package or converting an old one to catkin,
several tasks often need to be solved.  These pages give examples for
handling some common ones.

Overview
--------

.. toctree::
   :maxdepth: 1

   catkin_overview

Resolving dependencies
----------------------

Packages almost always use features provided by other packages.
Describe all your direct dependencies.  Their transitive dependencies
on other packages are handled automatically by catkin.

.. toctree::
   :maxdepth: 1

   catkin_library_dependencies
   system_library_dependencies
   cpp_msg_dependencies
   python_module_dependencies


Building and installing targets
-------------------------------

*Build targets* are generated binaries, shared libraries, message
headers, and other objects.  Various targets require special handling.

.. toctree::
   :maxdepth: 1

   building_executables
   building_libraries
   building_msgs
   dynamic_reconfiguration
   installing_python
   installing_cmake
   installing_other

