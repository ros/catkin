Catkin workspace layout
=======================

A Catkin workspace is a folder with a CMakeLists.txt file. It acts
like one huge cmake project containing any cmake project that you 
add as subfolder as a subproject.

.. contents::


``src`` (the Workspace folder)
------------------------------

You can place your workspace folder wherever you want and name it
what you want. One standard layout for ROS is to have a folder named
like the distro you use this workspace for, and in that folder a
folder named ``src``, to use as workspace folder.

In the ``src`` folder, you can use
`rosws <http://www.ros.org/doc/api/rosinstall/html/>`_ to download
and update catkin packages from multiple sources.

This directory must contain a symlink::

   CMakeLists.txt -> path/to/catkin/cmake/toplevel.cmake

which triggers catkin's topological sort via inspection of all
``package.xml`` files in the workspace.

Package
-------

A typical catkin "package" contains, at the top level::

   package.xml
   CMakeLists.txt

.. _package.xml:

package.xml
^^^^^^^^^^^

Used by catkin's dependency-sorting to determine what order to
traverse CMake projects in (when in a catkin workspace) and by
packaging utilities when making Debian packages.

The specification for package.xml is in `REP 127 <http://www.ros.org/reps/rep-0127.html>`_.

CMakeLists.txt
^^^^^^^^^^^^^^

The basic case::

   cmake_minimum_required(VERSION 2.8.3)
   project(pkgname)

   find_package(catkin REQUIRED [COMPONENTS otherpkg ...])
   # find_package(...)

   catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ${PROJECT_NAME}
      DEPENDS otherpkg)

   find_package(Boost REQUIRED COMPONENTS date_time thread)

   include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
   link_directories(${catkin_LINK_DIRS})

   set(${PROJECT_NAME}_SRCS
      src/duration.cpp
      src/rate.cpp
      src/time.cpp
   )

   add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SRCS})

   target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

   catkin_python_setup()

   install(TARGETS ${PROJECT_NAME}
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )

   install(DIRECTORY include/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   )

   # register unit tests

.. rubric:: cmake_minimum_required

The leading ``cmake_minimum_required`` is standard CMake.  Not
necessary when building in a workspace (as the first CMakeLists.txt
has already been read), but necessary when building e.g. in a
packaging context.

.. rubric:: project

This is standard CMake and sets the name of the package.  It must
match the name of the package specified in the ``package.xml`` file.

.. rubric:: find_package

This is standard CMake to find catkin and optionally any other catkin
package specified under COMPONENTS.  Instead of the more succinct
COMPONENTS method you could also use individual ``find_package``
calls for each catkin component.

Additionally ``find_package`` whatever else is necessary. Consider
using `REQUIRED <standards.html#find-package-required>`_ whenever
possible.

The variables ``find_package`` defines are also standard CMake.

.. rubric:: catkin_package

It declares what dependent packages need as include directories,
libraries and transitive dependencies which is used to generate
find_package and pkg-config infrastructure code.  Furthermore it
parses the ``package.xml`` and provides some of the information as
CMake variables.

.. rubric:: include_directories

This is standard CMake.  You will want to include
``${catkin_INCLUDE_DIRS}`` and other folders where necessary.

.. rubric:: add_library

Using ``${PROJECT_NAME}`` wherever possible to avoid repeating the
project name.  This is standard CMake.  Explicitly use ``SHARED`` for
building a shared library.

.. rubric:: target_link_libraries

Using ``${PROJECT_NAME}`` wherever possible to avoid repeating the
project name.  This is standard CMake.  Explicitly link against all
necessary libraries, i.e. ``catkin_LIBRARIES``.

.. rubric:: catkin_python_setup

Call :cmake:macro:`catkin_python_setup` if the project contains a
setup.py / Python code which should installed.

.. rubric:: install

This is standard CMake whitelisting which files should be installed.
Install all targets and resources as necessary.  The catkin provided
variables should be used to identify the install destinations.
