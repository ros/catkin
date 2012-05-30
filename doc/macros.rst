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

.. cmake:macro:: catkin_stack()

   `Required for all stacks.  No parameters.`

   Reads the :ref:`stack.xml` from the current source dir and makes
   the version number available to cmake via ``stackname_VERSION``;
   does the same for other fields in the `stack.xml`.  Also sets
   ``CATKIN_CURRENT_STACK``.  You must call `catkin_stack()` once
   in each stack's CMakeLists.txt, before any calls to `catkin_project()`,
   to ensure that auto-generated pkg-config and CMake files get correct
   version information.

.. cmake:macro:: catkin_workspace()

   `No parameters.`

   Called only in catkin's ``toplevel.cmake``, normally symlinked to
   from the workspace level directory (which contains multiple
   stacks).  This provokes the traversal of the stack directories
   based on the dependencies specified in the ``build_depends`` field of
   their respective ``stack.xml`` files.

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


Testing macros
^^^^^^^^^^^^^^

.. cmake:macro:: initialize_tests()

   Initialize.  Tests.

.. cmake:macro:: append_test_to_cache(CACHENAME [args])

   `Internal use.`

   :param CACHENAME: Name of cache.
   :param [args]:    Command to be appended to cache file.

   Use this when you want to append to a file that is recreated at
   each cmake run.  ``CACHENAME`` need not be globally unique.  File
   will be located in the ``PROJECT_BINARY_DIR`` cmake files directory
   (`CMakeFiles`) as ``${PROJECT_NAME}.${CACHENAME}``.

.. cmake:macro:: add_pyunit(FILE)

   :param FILE: name of pyunit test file

   Add file to test list and run under `rosunit` at testing time.


.. cmake:macro:: add_gtest(EXE FILES [parameters])

   :param EXE: executable name
   :param FILES: list of gtest .cpp files
   :param TIMEOUT: The timeout in seconds (defaults to 60s)
   :param WORKING_DIRECTORY: The working directory

   Add an executable `EXE` build from `FILES` and link to gtest.  Run under
   `rosunit` when test target is built.


Convenience macros
^^^^^^^^^^^^^^^^^^

.. cmake:macro:: install_matching_to_share(globexpr)

   :param globexpr: Glob expression (shell style)

   For each file `F` in subdirectories of ``CMAKE_CURRENT_SOURCE_DIR``
   that (recursively) match globbing expression `globexpr`, install
   `F` to ``share/P/F``, where ``P`` is the name of the parent
   directory of `F`

   .. rubric:: Example

   For a directory containing::

     src/
       CMakeLists.txt
       foo/
         bar.txt
       shimmy/
         baz/
           bam.txt

   A call to ``install_matching_to_share(b??.txt)`` in
   ``src/CMakeLists.txt`` will create an installation of::

     <CMAKE_INSTALL_PREFIX>/
       share/
         foo/
           bar.txt
         baz/
           bam.txt


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


.. cmake:macro:: stamp(filepath)

   :param filepath:  file name

   Use ``configure_file`` to generate a file ``filepath.stamp`` hidden
   somewhere in the build tree.  This will cause cmake to rebuild its
   cache when ``filepath`` is modified.
