Catkin cmake macro reference
============================

.. cmake:macro::  catkin_project(projectname [parameters])

   :param projectname: requires the same value as passed to cmake's ``project()``
   :param INCLUDE_DIRS: source-relative path to C/C++ headers
   :param LIBRARIES: names of library targets
   :param MSG_DIRS: source-relative paths to directories containing messages
   :param CFG_EXTRAS: Any extra cmake stuff that should be accessible
      to users of the project.  This file should live in subdirectory
      'cmake' and have extension ``.in``.  It will be expanded by cmake's
      ``configure_file()`` and made available to clients in both the
      install and build spaces: be sure it works both ways (by checking
      ``proj_SOURCE_DIR``, which is not set in the install case)
   :outvar PROJECT_SHARE_INSTALL_PREFIX: set to
      ``CMAKE_INSTALL_PREFIX/share/${PROJECT_NAME}`` by default.  For
      use with cmake ``install()`` macro.

   Create buildspace :term:`config-mode infrastructure`.  This allows

   .. rubric:: Example

   ::

     catkin_project(proj
       INCLUDE_DIRS include
       LIBRARIES proj-one proj-two
       MSG_DIRS msg
       CFG_EXTRAS proj-extras.cmake
       )

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

   `Optional.  No parameters.`

   Reads the :ref:`stack.yaml` from the
   current source dir and makes the version number available to cmake
   via ``stackname_VERSION``.

.. cmake:macro:: catkin_workspace()

   `No parameters.`

   Called only in catkin's ``toplevel.cmake``, normally symlinked to
   from the workspace level directory (which contains multiple
   stacks).  This provokes the traversal of the stack directories
   based on the dependencies specified in the ``Depends`` field of
   their respective ``stack.yaml`` files.

Documentation Macros
^^^^^^^^^^^^^^^^^^^^

.. cmake:macro:: sphinx(SOURCEDIR BUILDDIR)

   :param SOURCEDIR:  Directory containing sphinx .rst documentation source code
   :param BUILDDIR:   Directory to contain generated html
   :target <PROJECT_NAME>-sphinx:  Builds html documentation.  Dependee:  toplevel target ``doc``

   Optionally creates ``-deploy`` targets, see :cmake:data:`CATKIN_DOCS_DEPLOY_DESTINATION`.

.. cmake:macro:: find_sphinx()

   :outvar SPHINX_BUILD: Path to ``sphinx-build`` binary.

   Finds sphinx binary.  You don't need this... called automatically by :cmake:macro:`sphinx()`

.. cmake:data:: CATKIN_DOCS_DEPLOY_DESTINATION

   :default: ``OFF``

   If  this is set, the  ``*-sphinx``  targets above  will also  have
   ``*-sphinx-deploy``  targets which rsync  the documentation  to the
   provided  location  (value  may  contain ``user@``:  it  is  passed
   directly to cmake)


Macros pulled in from project genmsg
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

*These docs should move to genmsg*

.. cmake:macro:: generate_messages([parameters])

   :param optional DEPENDENCIES: names of projects that the messages in this
      package depend on.

   :param optional LANGS: generate messages for these languages.
      This will fail if you specify messages that catkin doesn't know
      about.  More appropriate use: to prevent generation for certain
      languages.

   This is actually defined in package ``genmsg``, should be documented there.


.. cmake:macro:: add_message_files(...)

   :param path DIRECTORY: source-relative path to directory containing messages
   :param list FILES: paths to files relative to ``DIRECTORY`` parameter





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


.. cmake:macro:: add_gtest(EXE FILES)

   :param EXE: executable name
   :param FILES: list of gtest .cpp files

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


.. cmake:macro:: catkin_add_env_hooks(buildspace_file installspace_file)

   :param buildspace_file:  environment file (bourne-shell syntax)
     to be expanded and added to build environment
   :param installspace_file:  as above, but installation version

   Find ``buildspace_file.in`` and expand.  Place where it will be
   read by generated ``setup.sh`` and friends.

   Similarly, install expanded ``installspace_file.in`` where it will
   be read by the installed ``setup.sh`` and friends.


