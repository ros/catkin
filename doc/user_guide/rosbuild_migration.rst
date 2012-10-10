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
along a rising tide.

Quick start guide
.................

At a high level, the key steps to convert a ROS stack from rosbuild to
catkin packages are:

1. Create a catkin workspace to hold your migrated projects
2. Move any project to this catkin workspace once all its dependencies are catkinized
3. Convert your stack into a meta-package, unless it was a unary rosbuild stack (Unary rosbuild stacks are stacks with just one package, those will become one catkin package)

 2. create a catkin package in your old stack folder and name it like the stack, this is going to be the meta-package (as an example, see ros_comm/ros_comm)
 3. add all other packages of the old stack as run_depend tags of the meta-package in the package.xml
 4. remove the stack.xml, the information from it moves into the package.xml of the meta-package
 5. What previously was your 'stack' folder should not contain only packages and at most one meta-package.

5. For each folder containing a ``manifest.xml`` file:

 a. rename the ``manifest.xml`` to ``package.xml``
 b. add a name tag with the name of the package, which should also be the folder name
 c. If missing, create a CMakeLists.txt file containing a ``catkin_project()`` invocation

4. In each ``CMakeLists.txt``:

 a. If rosbuild macros were used, switch from rosbuild macros to the underlying CMake commands
 b. Declare how your targets (c++ binaries) shall be installed

Detailed migration guide
........................

rosbuld wrapped several CMake macros with macros named
``rosbuild_...``. Catkin attempts to not wrap commands,
so you will be using the standard cmake macros, with only
a few added catkin macros if necessary.

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
