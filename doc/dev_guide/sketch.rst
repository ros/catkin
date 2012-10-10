Design sketch
=============

* There is only one invocation of CMake (and make) to build a source
   directory containing N packages.

* There is no search of ROS_PACKAGE_PATH: the packages to be built
   are the subdirectories (potentially recursive) of the
   ``CMAKE_SOURCE_DIR``.

* Catkin does only knows about packages: it simply examines the
   subdirectories under ``CMAKE_SOURCE_DIR`` for buildable
   projects (indicated by the presence of a file ``package.xml`` (see
   `REP 127 <http://www.ros.org/reps/rep-0127.html>`_).  It determines
   the dependency ordering of packages by examining these files.

* The ability to build only specific targets or sets of targets are
   provided through cmake's "natural" mechanisms
   (Meaning ``$ make foo`` only builds target foo)

* The build does not modify the source directories in any way.

* The build does not depend on any compiled code (i.e. ``rospack``)
   for ease of compiling on windows, cross-compiling for ARM, etc.
   (but may depend on python scripts)

* Packages use standard CMake macros, modulo a few that are provided
   by catkin.

* The build system depends only on CMake and Python.

* Downloading, untarring, patching and especially the wanton use of
   ``sed`` and/or handcoded Makefiles is considered to be in the very
   poorest taste.  Exception: for now, Sphinx-generated Makefiles used
   to generate documentation are okay.

* The build process creates a buildpace folder that has the same
  layout as an installation, so development cycles do not need to
  invoke ``make install`` frequently, if the developer decides
  to use that buildspace directly.

Main trickery
-------------

.. rubric:: Dependency ordering of packages

During the CMake run the main source directory is scanned for
packages to build. Each package indicates that it is buildable, and
what its dependencies in the ``package.xml`` file.  Catkin considers
the ``build`` and ``buildtool`` dependencies as specified in  the
`REP 127 <http://www.ros.org/reps/rep-0127.html>`_, topologically
sorts the packages and calls ``add_subdirectory()`` on each package
in order.

.. rubric:: Generation of ``find_package`` infrastructure

Each package calls :cmake:macro:`catkin_package` which generates
``find_package`` files; this is for other packages to use.  Each
package does a ``find_package`` of each other; in this way, packages
can build with their dependencies in the same buildspace, or already
installed on the system.  Different versions of these files are
created for use in the buildspace, and in the installation.  The
stress-test for this scheme is message generation; different language
generators must implement a specific CMake interface which is used by
``genmsg``.  See also :ref:`find_package_internals`.

.. rubric:: Python path forwarding

A special thunk (via the standard python package `pkgutil
<http://docs.python.org/library/pkgutil.html>`_ is generated and put
in
``CMAKE_BINARY_DIR/lib/pythonX.Y/dist-packages/PACKAGENAME/__init__.py``,
which extends the ``PYTHONPATH`` to include
``CMAKE_SOURCE_DIR/path/to/PACKAGENAME/src``.  This way the
buildspace PYTHONPATH only needs to contain
``CMAKE_BINARY_DIR/lib/pythonX.Y/dist-packages``.  Caveat: it will
also need to contain generated Python code in that folder in
buildspace.  At installation time, this thunk-__init__.py disappears
and the static python library source is installed alongside the
generated message code in the same directory.

.. rubric:: Environment generation

When CMake runs, it knows the locations of the ``CMAKE_BINARY_DIR``
and so forth, it generates an environment file (in
``CMAKE_BINARY_DIR/buildspace``).  Projects may extend this environment via
:cmake:macro:`catkin_add_env_hooks`.
