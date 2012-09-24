Migrating from Fuerte catkin
============================

Catkin still changes a lot, so a migration from catkin in fuerte to
catkin in groovy is necessary. Changes to catkin in Groovy are not
designed to be backwards compatible to catkin in Fuerte.

To update an already catkinized ROS stack from the Fuerte-version of catkin to Groovy the following steps are necessary:

Make a meta-package for the stack.
----------------------------------

The latest version of Catkin does not support the idea of a "stack" of packages which get wrapped up together into a
Debian (or similar) installable package.  Instead, catkin packages now get built into separate installable Debian
(or similar) packages.  To maintain compatibility with software which has a dependency on the stack name, you need to
make a new catkin package which is a "metapackage".  This package has the same name as the stack and contains no
source code, only a list of dependencies containing all the packages which used to be in the stack.

For example, say our old stack was named "stick" and it had 2 packages: "pickage" and "peckage".  The directories
would look like this::

  /stick/stack.xml
  /stick/CMakeLists.xml
  /stick/pickage/... (pickage files)
  /stick/peckage/... (peckage files)

The new version adds a new "stick" metapackage subdirectory under /stick, along with a package.xml and CMakeLists.txt
file::

  /stick/stick/package.xml
  /stick/stick/CMakeLists.txt
  /stick/pickage/... (pickage files)
  /stick/peckage/... (peckage files)

The contents of the /stick/stick/CMakeLists.txt file should look like this::

  cmake_minimum_required(VERSION 2.8)
  project(stick)
  find_package(catkin REQUIRED)
  catkin_package()

Note that the name of the metapackage ("stick") is stored once in here, but no other changes are needed for
a metapackage.

Then /stick/stick/package.xml looks like this::

  <package>
    <name>stick</name>
    <version>0.1.1</version>
    <description>Meta-package relevant to sticks.</description>
    <maintainer email="coder@example.com">Lucy Coder</maintainer>
    <license>BSD</license>

    <url type="website">http://www.example.com/stick</url>

    <build_depend>pickage</build_depend>

    <!-- a former stack depends only on all its child packages -->
    <run_depend>pickage</run_depend>
    <run_depend>peckage</run_depend>

    <export>
      <metapackage/>
    </export>
  </package>

The rest of these instructions refer to changes within regular (not meta-) catkin packages.

Update `CMakeLists.txt`` files
------------------------------

* Instead of ``find_package(ROS ...)`` use ``find_package(catkin ...)``.

  Make sure to update the corresponding variables like ``ROS_INLCUDE_DIRS`` to ``catkin_INCLUDE_DIRS``.

  Do not search a component called ``catkin``.

* ``catkin_package(...)`` should be called near the beginning of the CMake, directly after ``find_package``-ing catkin and other dependencies.
  Unlike the old ``catkin_project()`` macro, ``catkin_package()`` doesn't need the name of the package it is part of.

* Switch to renamed catkin functions:

  * ``add_gtest`` => ``catkin_add_gtest``

    Do not use path-like string for the target name.
    The first argument must be a valid CMake target name.

  * ``add_nosetests`` => ``catkin_add_nosetests``

* Update install() invocations to use the new FHS compliant destinations (see :ref:`variables`).
  Always specify the necessary destinations explicitly.

  Specify ``DESTINATION``, ``ARCHIVE DESTINATION``, ``LIBRARY DESTINATION`` and ``RUNTIME DESTINATION`` as required.

* Remove manually ``install()`` invocations for ``stack.xml`` and ``manifest.xml`` files (this is handled by catkin automatically).

* After creating a GTest target using ``catkin_add_gtest(target ...)`` you should test for the existance of the target before trying to use it (i.e. by calling ``target_link_libraries(target ..,)``)::

  % if(TARGET target)
  %   target_link_libraries(target ...)
  % endif()

  This handles the case gracefully when GTest is not available.

CMake extra files
-----------------

CMake extra files must now work in buildspace as well as in installspace.
The templates can determine the different invocation cases using the variables ``@BUILDSPACE@`` and ``@INSTALLSPACE@``.

Custom find_package() config files
----------------------------------

The ``find_package()`` config have been renamed from ``<projectname>-config.cmake.in`` to ``<ProjectName>Config.cmake.in``.
Note that the project name is no longer converted to lower case but used as-is.

Custom environment hooks
------------------------

The names of the templates for the environment hooks for buildspace and installspace have been unified.
There is only one template for both.
The templates can determine the different invocation cases using the variables ``@BUILDSPACE@`` and ``@INSTALLSPACE@``.
