Catkin cmake macro reference
============================

.. cmake:macro::  catkin_project(projectname [parameters])

   It creates the cmake stuff necessary for ``find_package`` to work
   (i.e. to be *found* by others that call ``find_package``) and
   provide information about include directories, libraries,
   further dependencies and cmake variables for dependent projects.

   Best practice is to call this macro early in your CMakeLists.txt,
   immediately after calling ``project()``.

   :param projectname: requires the same value as passed to cmake's
      ``project()`` (*may be vestigial*)
   :param INCLUDE_DIRS: ``CMAKE_CURRENT_SOURCE_DIR``-relative path to
      C/C++ includes
   :param LIBRARIES: names of library targets that will appear in the
      ``ROS_LIBRARIES`` of other projects that search for you via
      ``find_package``.  Currently this will break if the logical
      target names are not the same as the installed names.
   :param DEPENDS: The argument ``DEPENDS`` is used when client code
      finds us via ``find_package()``.  Each project listed will in
      turn be ``find_package``\ -ed and their ``INCLUDE_DIRS`` and
      ``LIBRARIES`` will be appended to ours.  Only catkin projects
      should be used where we can ensure that they are
      *find_packagable* and *package_configurable*.
   :param CFG_EXTRAS: Any extra cmake stuff that should be accessible
      to users of the project.  This file should live in subdirectory
      ``cmake`` and have extension ``.in``.  It will be expanded by
      cmake's ``configure_file()`` and made available to clients in
      both the install and build spaces: be sure it works both ways
      (by checking ``proj_SOURCE_DIR``, which is not set in the
      install case)
   :outvar PROJECT_SHARE_INSTALL_PREFIX: set to
      ``CMAKE_INSTALL_PREFIX/share/${PROJECT_NAME}`` by default.  For
      use with cmake ``install()`` macro.

   .. rubric:: Example

   ::

     catkin_project(proj
       INCLUDE_DIRS include
       LIBRARIES proj-one proj-two
       DEPENDS roscpp
       CFG_EXTRAS proj-extras.cmake
       )

.. index:: setup.py

.. cmake:macro:: catkin_export_python([dir1 dir2 ...])

   :param dirs: list of directories given relative to
      CMAKE_CURRENT_SOURCE_DIR containing python code

   Creates forwarding python :term:`pkgutil` infrastructure that "points"
   from the part of the build directory that holds :term:`generated
   code` (python) back to the specified source directories that hold
   :term:`static code`.

   In addition, this will provoke that the python distutils' file
   ``setup.py`` is interrogated by catkin and used during
   installation.  See :ref:`setup_dot_py_handling`.

Documentation Macros
^^^^^^^^^^^^^^^^^^^^

.. cmake:macro:: catkin_sphinx(SOURCEDIR BUILDDIR)

   :param SOURCEDIR:  Directory containing sphinx .rst documentation source code
   :param BUILDDIR:   Directory to contain generated html
   :target <PROJECT_NAME>-sphinx:  Builds html documentation.  Dependee:  toplevel target ``doc``

   Optionally creates ``-deploy`` targets, see :cmake:data:`CATKIN_DOCS_DEPLOY_DESTINATION`.

.. cmake:macro:: find_sphinx()

   :outvar SPHINX_BUILD: Path to ``sphinx-build`` binary.

   Finds sphinx binary.  You don't need this... called automatically by :cmake:macro:`catkin_sphinx()`

.. cmake:data:: CATKIN_DOCS_DEPLOY_DESTINATION

   :default: ``OFF``

   If  this is set, the  ``*-sphinx``  targets above  will also  have
   ``*-sphinx-deploy``  targets which rsync  the documentation  to the
   provided  location  (value  may  contain ``user@``:  it  is  passed
   directly to cmake)


.. cmake:macro:: catkin_add_env_hooks(fileprefix SHELLS shell1 shell2...)

   :param fileprefix: prefix of environment file to be expanded and
     added to build environment
   :param SHELLS:  list of shells

   For each shell in ``SHELLS``, find file
   ``<fileprefix>.buildspace.<shell>.in`` in the current directory and
   expand to ``CMAKE_BINARY_DIR/etc/catkin/profile.d/``, where it will
   be read by generated ``setup.<shell>``.

   Similarly, install expanded ``<fileprefix>.<shell>.in`` to
   ``CMAKE_INSTALL_PREFIX``/etc/catkin/profile.d, where it will be
   read by the installed ``setup.<shell>`` and friends.

   .. note:: Note the extra ".in" that must appear in the filename
      that does not appear in the argument.

   You my also specify ``all`` as a shell; this will be read by all
   shells, before the shell-specific files are read.  Note that your
   syntax had better be portable across all shells.

   **NOTE** These files will share a single directory with other
   packages that choose to install env hooks.  Be careful to give the
   file a unique name.  Typically ``NNprojectname.sh`` is used, where
   NN can define when something should be run (the files are read in
   alphanumeric order) and ``projectname`` serves to disambiguate in
   the event of collision.




