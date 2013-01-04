Packages
========

.. highlight:: catkin-sh

A catkin package is a folder with a valid ``package.xml`` and a
``CMakeLists.txt`` file (unless it is a meta-package).

.. _package.xml:

package.xml
^^^^^^^^^^^

This is used by catkin's dependency-sorting to determine what order to
traverse CMake projects in (when in a catkin workspace) and by
packaging utilities when making Debian packages.

The specification for package.xml is in `REP 127 <http://www.ros.org/reps/rep-0127.html>`_.

A small example is::

  <?xml version="1.0"?>
  <package>
    <name>example</name>
    <version>0.0.1</version>
    <description>An example package</description>
    <maintainer email="JohnDoe@ros.com">John Doe</maintainer>
    <license>BSD</license>
    <url type="website">http://www.ros.org/wiki/example</url>
    <author>John Doe</author>
    <build_depend>ros</build_depend>
    <build_depend>roscpp</build_depend>
    <run_depend>ros</run_depend>
    <run_depend>roscpp</run_depend>
  </package>

The version tag is mandatory and needs to be in the format
"decimal.decimal.decimal". The name should always match the folder
name of the package. A maintainer with an email address is also
required for the release process.

.. rubric:: build_depend

List here any other catkin package or system library required to build
this package. For catkin packages, this will determine the order they
get configured. For system libraries, this will allow to help other
installing the prerequisistes.

.. rubric:: run_depend

Similar to build_depend, but for packages and libraries that are not
needed at build time.

.. _cmakelists.txt:

CMakeLists.txt
^^^^^^^^^^^^^^

A minimal CMakeLists.txt ::

   cmake_minimum_required(VERSION 2.8.3)
   project(pkgname)

   find_package(catkin REQUIRED [COMPONENTS otherpkg ...])
   # find_package(...)

   ## Uncomment this if the package has a setup.py. This macro ensures
   ## modules and scripts declared therein get installed
   # catkin_python_setup()

   ## LIBRARIES: libraries you create in this project that dependent projects also need
   ## CATKIN_DEPENDS: catkin_packages dependent projects also need
   ## DEPENDS: system dependencies of this project that dependent projects also need
   catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ${PROJECT_NAME}
      CATKIN_DEPENDS catkinpkg1 catkinpkg2
      DEPENDS systempkg1 systempkg2)

   ...

   # install the library
   install(TARGETS ${PROJECT_NAME}
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )

   # install the header files
   install(DIRECTORY include/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   )


.. rubric:: cmake_minimum_required

The leading ``cmake_minimum_required`` is standard CMake.  Not
necessary when building in a workspace (as the first CMakeLists.txt
has already been read), but necessary when building e.g. in a
packaging context.

.. rubric:: project

This is standard CMake and sets the name of the package.  It must
match the name of the package specified in the ``package.xml`` file,
and must be called before the catkin_package() macro.

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

API doc: :ref:`catkin_package_ref`

.. rubric:: catkin_python_setup

Call :cmake:macro:`catkin_python_setup` if the project contains a
setup.py / Python code which should installed.

API doc: :ref:`catkin_python_setup_ref`


.. rubric:: install

This is standard CMake whitelisting which files or directories should
be installed. Install all targets and resources as necessary.  The
catkin provided variables should be used to identify the install
destinations, in the example these were::

   CATKIN_PACKAGE_BIN_DESTINATION
   CATKIN_PACKAGE_LIB_DESTINATION
   CATKIN_PACKAGE_INCLUDE_DESTINATION

Resources, assets, launchfiles, etc get installed using similar
destination variables (:ref:`variables`).
