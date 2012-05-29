Migrating from rosbuild to catkin
=================================

**UNDER CONSTRUCTION**

From Box Turtle to Fuerte, the ROS build system was `rosbuild
<http://ros.org/wiki/rosbuild>`_.  In Fuerte, we introduced `catkin
<http://ros.org/wiki/catkin>`_.  We aim to deprecate rosbuild in Groovy,
and encourage users to migrate their code from rosbuild to catkin.  We
especially encourage maintainers of released ROS stacks to migrate as soon
as possible, because the build farm that generates binary Debian packages
from released ROS stacks will eventually be decommissioned.

Quick start guide
.................

At a high level, the key steps to convert a ROS stack from rosbuild to
catkin are:

1. Update ``stack.xml``...
2. Create a top-level ``CMakeLists.txt`` for the stack...
3. Delete any ``manifest.xml`` files:

 a. Make each containing directory a CMake project...
 b. Combine ``rosdep`` keys from ``manifest.xml`` files into ``stack.xml`` as ``depends`` keys...

4. In each ``CMakeLists.txt``:

 a. Switch from rosbuild macros to the underlying CMake commands

Background
..........

What is a build system?
-----------------------

The main job of a build system is to declare what files should be
processed, in what way, to generate, compile, link, and/or install a chunk
of code.  As it processes files, the build system is specifically
responsible for managing dependencies: it must understand which parts of the
system depend which other parts, and process them in the appropriate order.

What is rosbuild?
-----------------

rosbuild is a `CMake <http://www.cmake.org/>`_-based build system that
aims to simplify the process of generating, compiling, and linking code in
ROS packages.  The intended user of rosbuild is a ROS package developer,
who "lives" in a ROS source tree, constantly making changes to code and
dependency structures, recompiling, and running the resulting programs.
Because of the source-tree use-case, rosbuild does not provide an
``install`` target; there is no (clean) way to install a
rosbuild-controlled package.

rosbuild works by wrapping standard CMake commands (e.g.,
``add_library()``) in custom macros (e.g., ``rosbuild_add_library()``)
that do extra steps (e.g., add libraries that were exported from other
packages) to ensure that the right thing happens.  In other words, there's
a lot of magic going on behind the scenes with rosbuild: you declare
dependencies in your package ``manifest.xml``, and rosbuild (pretty much)
does the rest.

What is catkin?
---------------

catkin is also a `CMake <http://www.cmake.org/>`_-based build system
that aims to simplify the process of generating, compiling, and linking
code in ROS packages.  But the intended user of catkin is three-fold: a
ROS package developer who lives in a ROS source tree, a release system (see
`bloom <http://ros.org/wiki/bloom>`_) that need to generate system-specific
binary installations, and an end-user who wants to use an installed ROS
system.

catkin works by adding some macros and functions, and does not wrap
standard CMake commands.  It does relatively litle magic, preferring to
make things explicit.  And it supports an ``install`` target.

What's different between rosbuild and catkin?
.............................................

Main differences between rosbuild and catkin:

- rosbuild has no ``install`` target; catkin does

 - as a result: catkin stacks must declare, in a whitelist fashion, what programs and files will be installed

- rosbuild relies heavily on ``bash`` and ``make``; catkin uses only CMake and Python (and therefore is more portable)
- rosbuild wraps standard CMake commands; catkin does not

 - as a result: rosbuild implicitly adds compile and link flags based on package dependency information; catkin requires you to add them explicitly

- rosbuild always does in-source builds; catkin supports both in-source and out-of-source builds (out-of-source is recommended)
- rosbuild uses ``manifest.xml`` for per-package dependency information; catkin uses ``stack.xml`` for per-package dependency information
- rosbuild use per-package dependency information to assemble compile and link flags; catkin uses per-stack dependency only to determine traversal order
- rosbuild provides a ``make`` interface to each package; catkin provides a ``cmake`` interface to each stack

 - as a result: rosbuild packages are built one-by-one, each with a separate invocation of ``make``; catkin stacks can be built in a fully parallel fashion, with a single build command (which is often, but does not have to be ``make``)

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
  - ``rosbuild_add_boost_directories(); rosbuild_link_boost(target components)`` -> ``find_package(Boost COMPONENTS components); include_directories(${Boost_INCLUDE_DIRS}); target_link_libraries(target ${Boost_LIBRARIES})``
  - ``rosbuild_add_openmp_flags()`` -> ``find_package(OpenMP)``, then do other stuff
  - ``rosbuild_invoke_rospack()`` -> don't do this
  - ``rosbuild_find_ros_package()`` -> don't do this
  - ``rosbuild_find_ros_stack()`` -> don't do this
  - ``rosbuild_check_for_sse()`` -> look around online and find an example of how to find SSE
  - ``rosbuild_include(package module)`` -> ``include(module)`` (might require some initial work to find the path to the module)
  - ``rosbuild_add_lisp_executable()`` -> no support for this currently

 - Test macros:

  - rosbuild_add_gtest
  - rosbuild_add_gtest_labeled
  - rosbuild_add_gtest_future
  - rosbuild_add_gtest_build_flags
  - rosbuild_add_pyunit
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
