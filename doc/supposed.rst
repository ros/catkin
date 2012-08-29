Sketch of directory structure
=============================

How things ought to look, starting at the outermost level of the
source directory and moving down through stacks and packages.

Workspace
---------

This is a directory containing multiple stacks, e.g. one constructed
by ``rosinstall``.  This directory must contain a symlink::

  CMakeLists.txt -> catkin/cmake/toplevel.cmake

which triggers catkin's topological sort via inspection of ``stack.xml``.

Stack
-----

A typical "stack" will contain, at the top level::

  stack.xml
  CMakeLists.txt
  project1/
  project2/

.. _stack.xml:

stack.xml
^^^^^^^^^

Used by catkin's dependency-sorting to determine what order to
traverse cmake projects in (when in a catkin workspace) and by
packaging utilities when making debians.

Example::

  <stack>
    <name>ros_comm</name>
    <version>0.1.2</version>
    <description brief="ROS communications-related libraries and tools">ROS communications-related packages, including core client libraries (roscpp, rospy, roslisp) and graph introspection tools (rostopic, rosnode, rosservice, rosparam).</description>
    <author>Troy Straszheim</author>
    <author>Morten Kjaergaard</author>
    <author email="kwc@willowgarage.com">Ken Conley</author>
    <maintainer email="straszheim@willowgarage.com">Troy Straszheim</maintainer>
    <license>BSD</license>
    <copyright>willowgarage</copyright>

    <url>http://www.ros.org</url>
    <review status="Doc reviewed" notes="2009/6/10"/>

    <build_depends>genmsg</build_depends>
    <build_depends>libboost-thread-dev</build_depends>
    <build_depends>libboost-date-time-dev</build_depends>

    <depends>libboost-thread</depends>
    <depends>libboost-date-time</depends>
  </stack>

For more details on the individual tags see :ref:`stack_xml`.

CMakeLists.txt
^^^^^^^^^^^^^^

The basic case (``ros_comm``)::

  cmake_minimum_required(VERSION 2.8)

  # find_package(catkin REQUIRED)
  # catkin_stack()

  # optional find_package(... [REQUIRED])

  # optional catkin_python_setup()

  foreach(subdir
      cpp_common
      rostime
      roscpp_traits
      roscpp_serialization
      )
    add_subdirectory(${subdir})
  endforeach()


.. rubric:: cmake_minimum_required

The leading ``cmake_minimum_required`` is standard cmake.  Not
necessary when building in a workspace (as the first CMakeLists.txt
has already been read), but necessary when building e.g. in a
packaging context.

.. rubric:: catkin_stack

Find the catkin package first and then call :cmake:macro:`catkin_stack`.

.. rubric:: find_package [optional]

The ``# optional find_package`` line is for anything that is common to
all subprojects and not handled by catkin_stack.  Consider using
`REQUIRED <standards.html#find-package-required>`_ whenever possible.
This is standard cmake.

.. rubric:: catkin_python_setup

Call :cmake:macro:`catkin_python_setup` if the project contains a 
setup.py / Python code which should installed.

.. rubric:: add_subdirectory

Then ``add_subdirectory(P)`` for each package ``P``.  Here the
dependency ordering between packages is defined, i.e. if ``proj2``
refers to a target defined in ``proj1``, then ``proj1`` must come
first in the ordering.


package
-------

Each package (as added by ``add_subdirectory`` in the stack) Will
contain a ``CMakeLists.txt``.  Basic case::

  project(rostime)

  find_package(ROS REQUIRED COMPONENTS catkin cpp_common)
  include_directories(${ROS_INCLUDE_DIRS})

  find_package(Boost REQUIRED COMPONENTS date_time thread)

  include_directories(include)

  set(${PROJECT_NAME}_SRCS
    src/duration.cpp
    src/rate.cpp
    src/time.cpp
  )

  add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SRCS})

  target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${ROS_LIBRARIES})

  install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    )

  install(DIRECTORY include/
    DESTINATION include
    )

  catkin_project(${PROJECT_NAME}
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    )


.. rubric:: project

This is standard cmake.

.. rubric:: find_package [optional]

``find_package`` of whatever is necessary.  This is standard cmake.
Consider using `REQUIRED <standards.html#find-package-required>`_
whenever possible.
For ``ROS``, you may use the aggregate
``find_package(ROS COMPONENTS ...)`` method, this will be more
succinct than a bunch of individual ``find_package`` calls.

*Yes*, you should specify ``catkin`` in this list of packages.  There
may be users that do not build with catkin's macros but wish to use
include/link flags for ROS libraries.  You may want to
``find_package`` of stack-wide components up at the top level, and
then find_package more specific components in the packages that use
them.

You will want to ``include_directories(${ROS_INCLUDE_DIRS})``
and other folders where necessary.

.. rubric:: source files

Add all source files to a list.  For better readability one file per
line with `alphabetic order <standards.html#keep-lists-sorted>`_.

.. rubric:: add_library

Using ``${PROJECT_NAME}`` where ever possible to avoid repeating the
project name.  This is standard cmake.  Explicitly use ``SHARED`` for
building a shared library.

.. rubric:: target_link_libraries

Using ``${PROJECT_NAME}`` where ever possible to avoid repeating the
project name.  This is standard cmake.  Explicitly link against all
necessary libraries, i.e. ``ROS_LIBRARIES``.

.. rubric:: install

``install`` your targets as necessary.  Libraries go in ``DESTINATION
lib``, include directories in ``DESTINATION include``, and "private"
stuff in ``share/${PROJECT_NAME}/``, i.e. private binaries thereunder
in ``bin/``... whatever turns out to be compatible with rosbuild.

.. rubric:: catkin_project

:cmake:macro:`catkin_project` defines information dependent projects
(i.e. include directories, libraries to link against and depending 
projects).

The ``VERSION`` argument is vestigial.



