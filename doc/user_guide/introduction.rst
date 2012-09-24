Introduction
============

.. highlight:: catkin-sh

Catkin helps building ROS packages by leveraging
the CMake tool that has for years been the most used C++ build
framework. The focus of catkin is C++ and Python based packages.

Users which have experience with CMake should feel right at home
with catkin.

.. contents::


What does Catkin provide?
-------------------------

The ROS ecosystem has a philosophy of federated development. Many
teams around the world develop their own modular ROS
packages. Building, installing, packaging and deploying those packages
is a common chore, and the catkin macros make all these tasks easier,
and provide a standard so that it becomes easier to use code by other
teams.

By encouraging best practice standards, ROS packages should in the
future become more portable and maintainable. Catkin encourages to
adhere to

* The Filesystem Hierarchy Standard (FHS)
* The CMake standards and best practices

Catkin is also extensible, such as allowing automatic inclusion of
genmsg, the framework of defining ROS messages with references to
messages in other ROS packages.

Finally Catkin provides a way to compile C++ sources from several
independent packages faster than by using only CMake on each project
individually.


If you have used rosmake before using catkin
--------------------------------------------

The workflow and commands of catkin and rosmake are different.

The documentation provides both a migration guide for existing ROS
stacks and packages as well as a comparison of the workflow steps.

.. toctree::
   :maxdepth: 1

   rosbuild_migration

If you have used CMake before using catkin
------------------------------------------

To get all catkin benefits you will need to learn a few catkin macros
on top of CMake macros.

Also you need to be aware of some restrictions catkin imposes on
CMakeLists.txt files.

.. toctree::
   :maxdepth: 1

   standards


Background
----------

What is a build system?
.......................

The main job of a build system is to declare what files should be
processed, in what way, to generate, compile, link, and/or install a chunk
of code.  As it processes files, the build system is specifically
responsible for managing dependencies: it must understand which parts of the
system depend which other parts, and process them in the appropriate order.

What is rosbuild?
.................

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
...............

catkin is also a `CMake <http://www.cmake.org/>`_-based build system
that aims to simplify the process of generating, compiling, and linking
code in ROS packages.  But the intended user of catkin is three-fold: a
ROS package developer who lives in a ROS source tree, a release system (see
`bloom <http://ros.org/wiki/bloom>`_) that need to generate system-specific
binary installations, and an end-user who wants to use an installed ROS
system.

catkin works by adding some macros and functions to CMake, but does not wrap
standard CMake commands.  It does relatively litle magic, preferring to
make things explicit.  And it supports an ``install`` target.

What's different between rosbuild and catkin?
.............................................

Main differences between rosbuild and catkin:

- rosbuild has no ``install`` target; catkin does
- catkin stacks can be properly installed on a variety of operating systems
- catkin stacks must declare, in a whitelist fashion, what programs and files will be installed
- rosbuild relies heavily on ``bash`` and ``make``; catkin uses only CMake and Python (and therefore is more portable)
- rosbuild wraps standard CMake commands; catkin does not

 - as a result: rosbuild implicitly adds compile and link flags based on package dependency information; catkin requires you to add them explicitly

- rosbuild always does in-source builds; catkin supports both in-source and out-of-source builds (out-of-source is recommended)

- rosbuild uses ``manifest.xml``; catkin uses ``package.xml``
- rosbuild requires to export compile and link flags in the manifest manually; catkin declares exported include directories, libraries and dependencies in CMake using ``catkin_package()``
- rosbuild provides a ``make`` interface to each package; catkin provides a ``cmake`` interface to each package but enables to build multiple packages in a workspace at once

 - as a result: rosbuild packages are built one-by-one, each with a separate invocation of ``make``; catkin stacks can be built in a fully parallel fashion, with a single build command (which is often, but does not have to be ``make``)
