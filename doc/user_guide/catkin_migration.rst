Migrating from Fuerte catkin
============================

Catkin still changes a lot, so a migration from catkin in fuerte to
catkin in groovy is necessary. Changes to catkin in Groovy are not
designed to be backwards compatible to catkin in Fuerte.

To update an already catkinized ROS stack from the Fuerte-version of catkin to Groovy the following steps are necessary:

Update `CMakeLists.txt`` files
------------------------------

* Instead of ``find_package(ROS ...)`` use ``find_package(catkin ...)``.

  Make sure to update the corresponding variables like ``ROS_INLCUDE_DIRS`` to ``catkin_INCLUDE_DIRS``.

  Do not search a component called ``catkin``.

* ``catkin_project()`` must be called for each ROS package and that
  should be done directly after ``find_package``-ing catkin.

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
