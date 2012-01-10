How things ought to look, starting at the outermost level of the
source directory and moving down through stacks and packages.

workspace
---------

This is a directory containing multiple stacks, e.g. one constructed
by ``rosinstall``.  This directory must contain a symlink::

  CMakeLists -> catkin/toplevel.cmake

which triggers catkin's topological sort via inspection of ``stack.yaml``.

stack
-----

A typical "stack" will contain, at the top level::

  stack.yaml
  CMakeLists.txt
  project1/
  project2/

.. _stack.yaml:

stack.yaml
^^^^^^^^^^

Used by catkin's dependency-sorting to determine what order to
traverse cmake projects in (when in a catkin workspace) and by
packaging utilities whne making debians.

Example::

  Catkin-ProjectName: roscpp_core
  Version: 0.1.3
  Description: Willow Garage low level build system macros and infrastructure
  Author: Troy Straszheim <straszheim@willowgarage.com>, Morten Kjaergaard, Ken Conley
  Maintainer: Troy Straszheim <straszheim@willowgarage.com>
  Homepage: http://www.ros.org

  Catkin-CopyrightType:  willowgarage
  Catkin-DebRulesType: cmake

  Depends: genmsg,
           libboost-thread-dev,
           libboost-date-time-dev

  Release-Pull: git://github.com/wg-debs/roscpp_core.git
  Release-Push: git@github.com:wg-debs/roscpp_core.git


.. rubric:: Catkin-ProjectName

The name of the stack.  Should correspond to the name of the directory
that the code is checked out in, *if* this directory is being
traversed by catkin. *possibly vestigial*

.. rubric:: Version

Major.Minor.Patchlevel version.  Numeric only.  Used by deb packaging
utilities.

.. rubric:: Description, Author, Maintainer, Homepage

For future use.

.. rubric:: Catkin-CopyrightType

Type of copyright.  Used to provide copyright/license to deb packages.
Options:  ``willowgarage``.

.. rubric:: Catkin-DebRulesType

Type of debian rules file to use.  Options: ``cmake``, or
``python_distutils``.  See catkin/bin/em for the definitions.
*python_distutils possibly vestigial, cmake should work for most things*.

.. rubric:: Depends

List of rosdeps of this package.  Used to determine dependency
ordering of current workspace (when this exists) and expanded via
rosdep to construct debian control files.

.. rubric:: Release-Pull, Release-Push

*vestigial*.  Delete me.


CMakeLists.txt
^^^^^^^^^^^^^^

The basic case (``ros_comm``)::

  cmake_minimum_required(VERSION 2.8)

  # optional find_package

  foreach(subdir
      cpp_common
      rostime
      roscpp_traits
      roscpp_serialization
      )
    add_subdirectory(${subdir})
  endforeach()


The leading ``cmake_minimum_required`` is standard cmake.  Not
necessary when building in a workspace (as the first CMakeLists.txt
has already been read), but necessary when building e.g. in a
packaging context.

The ``# optional find_package`` line is for anything that is common to
all subprojects.  This is standard cmake.

Then ``add_subdirectory(P)`` for each package ``P``.  Here the
dependency ordering between packages is defined, i.e. if ``proj1``
refers to a target defined in ``proj2``, then ``proj1`` must come
first in the ordering.

.. rubric:: catkin_package()

`Vestigial`.  Replace me with :cmake:macro:`catkin_stack`.

.. rubric:: catkin_stack()  [optional]

See :cmake:macro:`catkin_stack`


package
-------

Each package (as added by ``add_subdirectory`` in the stack) Will
contain a ``CMakeLists.txt``.  Basic case::

  project(rostime)
  find_package(ROS COMPONENTS catkin cpp_common)

  include_directories(include)
  include_directories(${ROS_INCLUDE_DIRS})

  find_package(Boost COMPONENTS date_time thread)

  add_library(rostime SHARED
    src/time.cpp src/rate.cpp src/duration.cpp)

  target_link_libraries(rostime ${Boost_LIBRARIES} ${ROS_LIBRARIES})

  install(TARGETS rostime
    DESTINATION lib
    )

  install(DIRECTORY include/
    DESTINATION include
    )

  catkin_project(rostime
    VERSION 0.0.0
    INCLUDE_DIRS include
    LIBRARIES rostime
    )


Start with ``project()``.  This is standard cmake.  Follow with
``find_package`` of whatever is necessary; for ``ROS``, you may use
the aggregate ``find_package(ROS COMPONENTS ...)`` method, this will
be more succinct than a bunch of individual ``find_package`` calls.
*Yes*, you should specify ``catkin`` in this list of packages.  There
may be users that do not build with catkin's macros but wish to use
include/link flags for ROS libraries.  You may want to
``find_package`` of stack-wide components up at the top level, and
then find_package more specific components in the packages that use
them.   You will want to ``include_directories(${ROS_INCLUDE_DIRS})``
where necessary and use ``ROS_LIBRARIES`` with cmake's
``target_link_libraries()``.

``install`` your targets as necessary.  Libraries go in ``DESTINATION
lib``, include directories in ``DESTINATION include``, and "private"
stuff in ``share/${PROJECT_NAME}/``, i.e. private binaries thereunder
in ``bin/``... whatever turns out to be compatible with rosbuild.

``catkin_project`` creates the cmake stuff necessary for
``find_package`` to work (i.e. to be *found* by others that call
``find_package``.  The first argument is the project name (*may be
vestigial*).  The ``VERSION`` argument is vestigial.  The
``INCLUDE_DIRS`` argument is the ``CMAKE_CURRENT_SOURCE_DIR``
-relative path to any C++ includes.  ``LIBRARIES`` are the names of
targets that will appear in the ``ROS_LIBRARIES`` of other projects
that search for you via ``find_package``.  Currently this will break
if the logical target names are not the same as the installed names.




