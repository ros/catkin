Catkin workspace layout
=======================

A Catkin workspace is a folder with a CMakeLists.txt file. It acts
like one huge cmake project containing any cmake project that you 
add as subfolder as a subproject.

.. contents::


``src`` (the Workspace folder)
------------------------------

You can place your workspace folder wherever you want and name it what
you want. One standard layout for ROS is to have a folder named like
the distro you use this workspace for, and in that folder a folder 
named src, to use as workspace folder.

In the src folder, you can use `rosws <http://www.ros.org/doc/api/rosinstall/html/>`_ to download and
update catkin projects from multiple sources.

This directory must contain a symlink::

  CMakeLists.txt -> catkin/cmake/toplevel.cmake

which triggers catkin's topological sort via inspection of ``package.xml``.

Package
-------

.. todo:: The separation into packages and packages is going to be
   dropped, then this page needs to be changed accordingly.

A typical "package" contains, at the top level::

  package.xml
  CMakeLists.txt
  project1/
  project2/

.. _package.xml:

package.xml
^^^^^^^^^^^

Used by catkin's dependency-sorting to determine what order to
traverse cmake projects in (when in a catkin workspace) and by
packaging utilities when making debians.

The specification for package.xml is in `REP 127 <http://www.ros.org/reps/rep-0127.html>`_.

CMakeLists.txt
^^^^^^^^^^^^^^

The basic case (``ros_comm``)::

  cmake_minimum_required(VERSION 2.8)

  find_package(catkin REQUIRED)

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

.. todo:: This is not how the CMakeLists.txt of ros_comm actually looks like anymore.

.. rubric:: cmake_minimum_required

The leading ``cmake_minimum_required`` is standard CMake.  Not
necessary when building in a workspace (as the first CMakeLists.txt
has already been read), but necessary when building e.g. in a
packaging context.

.. rubric:: find_package(catkin REQUIRED)

This provides all catkin macros.

.. rubric:: find_package [optional]

The ``# optional find_package`` line is for anything that is common to
all subprojects.  Consider using
`REQUIRED <standards.html#find-package-required>`_ whenever possible.
This is standard CMake.

.. rubric:: catkin_python_setup

Call :cmake:macro:`catkin_python_setup` if the project contains a
setup.py / Python code which should installed.

.. rubric:: add_subdirectory

Then ``add_subdirectory(P)`` for each package ``P``.  Here the
dependency ordering between packages is defined, i.e. if ``proj2``
refers to a target defined in ``proj1``, then ``proj1`` must come
first in the ordering.


Package
-------

Each package (as added by ``add_subdirectory`` in the package)
contains a ``CMakeLists.txt``.

CMakeLists.txt
^^^^^^^^^^^^^^

Basic case::

  project(rostime)
  find_package(catkin REQUIRED COMPONENTS cpp_common)

  catkin_project(${PROJECT_NAME}
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    )

  include_directories(${catkin_INCLUDE_DIRS})
  link_directories(${catkin_INCLUDE_DIRS})

  include_directories(include)

  find_package(Boost REQUIRED COMPONENTS date_time thread)

  set(${PROJECT_NAME}_SRCS
    src/duration.cpp
    src/rate.cpp
    src/time.cpp
  )

  add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SRCS})

  target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

  install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    )

  install(DIRECTORY include/
    DESTINATION include
    )


.. rubric:: project

This is standard CMake.

.. rubric:: find_package [optional]

``find_package`` of whatever is necessary.  This is standard CMake.
Consider using `REQUIRED <standards.html#find-package-required>`_
whenever possible.
For ``catkin``, you may use the aggregate
``find_package(catkin COMPONENTS ...)`` method, this will be more
succinct than a bunch of individual ``find_package`` calls.

You may want to ``find_package`` of package-wide components up at the
top level, and then find_package more specific components in the
packages that use them.

.. rubric:: catkin_project

:cmake:macro:`catkin_project` defines information dependent projects
(i.e. include directories, libraries to link against and depending
projects).

You will want to ``include_directories(${ROS_INCLUDE_DIRS})``
and other folders where necessary.

.. todo:: more detail required here


.. rubric:: add_library

Using ``${PROJECT_NAME}`` wherever possible to avoid repeating the
project name.  This is standard CMake.  Explicitly use ``SHARED`` for
building a shared library.

.. rubric:: target_link_libraries

Using ``${PROJECT_NAME}`` wherever possible to avoid repeating the
project name.  This is standard CMake.  Explicitly link against all
necessary libraries, i.e. ``ROS_LIBRARIES``.

.. rubric:: install

``install`` your targets as necessary.  Libraries go in ``DESTINATION
lib``, include directories in ``DESTINATION include``, and "private"
stuff in ``share/${PROJECT_NAME}/``, i.e. private binaries thereunder
in ``bin/``... whatever turns out to be compatible with rosbuild.
