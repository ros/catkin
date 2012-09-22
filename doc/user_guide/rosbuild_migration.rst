Migrating from rosbuild to catkin
=================================

From Box Turtle to Fuerte, the ROS build system was `rosbuild
<http://ros.org/wiki/rosbuild>`_. In Fuerte, `catkin
<http://ros.org/wiki/catkin>`_ has been introduced for core ROS
stacks.

The goal is to deprecate rosbuild in the future, and encourage
users to migrate their code from rosbuild to catkin. We especially
encourage maintainers of released ROS packages to migrate early, because
the build farm that generates binary Debian packages from released ROS
stacks will eventually be decommissioned in the future.

Since catkinized packages cannot depend on rosbuild packages, the
migration has to start with core stacks and move on along the
dependency chain. This is the motivation for the wet/dry metaphor
alon a rising tide.

Quick start guide
.................

.. todo:: The separation into stacks and packages is going to be
   dropped, then this page needs to be changed accordingly.

At a high level, the key steps to convert a ROS stack from rosbuild to
catkin are:

1. Create a catkin workspace to hold your migrated projects
2. Move any project to catkin workspace once all their dependencies are catkinized

.. 3. In each stack, update the manifests (``stack.xml``...)
.. 4. Create a top-level ``CMakeLists.txt`` for the stack...

5. For each folder containing a ``manifest.xml`` file:

 a. Replace the exported explicit compiler and linker flags with a dynamic pkg-config invocation:

   ``<cpp cflags="`pkg-config --cflags PACKAGE_NAME`" lflags="`pkg-config --libs PACKAGE_NAME`"/>``

This allows rosbuild packages to depend on catkinized packages.

 b. Create a CMakeLists.txt file containing a ``catkin_project(PACKAGE_NAME)`` invocation...
 c. Combine ``rosdep`` keys from ``manifest.xml`` files into ``stack.xml`` as ``build_depends`` and ``depends`` keys...

4. In each ``CMakeLists.txt``:

 a. Switch from rosbuild macros to the underlying CMake commands

Detailed migration guide
........................

- Macro migration:

 - Build macros:

  - ``rosbuild_add_library(...)`` -> ``add_library(...)``
  - ``rosbuild_add_executable(...)`` -> ``add_executable(...)``
  - ``rosbuild_add_compile_flags(target added_flags)`` -> ``set_target_properties(target PROPERTIES COMPILE_FLAGS new_flags)``
  - ``rosbuild_remove_compile_flags(target removed_flags)`` -> ``set_target_properties(target PROPERTIES COMPILE_FLAGS new_flags)``
  - ``rosbuild_add_link_flags(target added_flags)`` -> ``set_target_properties(target PROPERTIES LINK_FLAGS new_flags)``
  - ``rosbuild_remove_link_flags(target removed_flags)`` -> ``set_target_properties(target PROPERTIES LINK_FLAGS new_flags)``
  - ``rosbuild_add_boost_directories(); rosbuild_link_boost(target components)`` -> ``find_package(Boost REQUIRED COMPONENTS components); include_directories(${Boost_INCLUDE_DIRS}); target_link_libraries(target ${Boost_LIBRARIES})``
  - ``rosbuild_add_openmp_flags()`` -> ``find_package(OpenMP)``, then do other stuff
  - ``rosbuild_invoke_rospack()`` -> don't do this
  - ``rosbuild_find_ros_package()`` -> don't do this
  - ``rosbuild_find_ros_stack()`` -> don't do this
  - ``rosbuild_check_for_sse()`` -> look around online and find an example of how to find SSE
  - ``rosbuild_include(package module)`` -> ``include(module)`` (might require some initial work to find the path to the module)
  - ``rosbuild_add_lisp_executable()`` -> no support for this currently

 - Test macros:

  - ``rosbuild_add_gtest(...)`` -> ``catkin_add_gtest(...)``
  - rosbuild_add_gtest_labeled
  - rosbuild_add_gtest_future
  - rosbuild_add_gtest_build_flags
  - ``rosbuild_add_pyunit`` -> migrate to ``catkin_add_nosetests(...)``
  - rosbuild_add_pyunit_labeled
  - rosbuild_add_pyunit_future
  - rosbuild_add_rostest
  - rosbuild_add_rostest_labeled
  - rosbuild_add_rostest_future
  - rosbuild_add_roslaunch_check
  - rosbuild_declare_test
  - rosbuild_count_cores
  - rosbuild_check_for_display
  - rosbuild_check_for_vm
