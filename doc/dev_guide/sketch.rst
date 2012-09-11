Design sketch
=============

* There is only one invocation of cmake (and make) to build a source
  directory containing N stacks.

* There is no search of ROS_PACKAGE_PATH: the stacks/packages to be
  built are the subdirectories (not recursive) of the
  ``CMAKE_SOURCE_DIR``.

* Catkin does not differentiate between stacks or packages: it simply
  examines the subdirectories of ``CMAKE_SOURCE_DIR`` for buildable
  projects (indicated by the presence of a file :ref:`stack.xml`).  It
  determines the dependency ordering of stacks by examining these
  files.

* The ability to build only specific targets or sets of targets are
  provided through cmake's "natural" mechanisms.

* The build does not modify the source directories in any way.

* The build does not depend on any compiled code (i.e. ``rospack``)
  for ease of compiling on windows, cross-compiling for ARM, etc.

* Packages use standard cmake macros, modulo a few that are provided
  by catkin.

* The build system depends only only on python and cmake.

* Downloading, untarring, patching and especially the wanton use of
  ``sed`` and/or handcoded Makefiles is considered to be in the very
  poorest taste.  Exception: for now, Sphinx-generated Makefiles used
  to generate documentation are okay.


Main trickery
-------------

.. rubric:: Dependency ordering of stacks

During the cmake run the main source directory is scanned for stacks
to build.  Actually cmake does not care if it is a stack or a package;
it cares that the directory has something there that indicates that
it is buildable, and what its dependencies are... that, is :ref:`stack.xml`.  Catkin
topologically sorts this and reads the files in order.

.. rubric:: Generation of ``find_package`` infrastructure

Each project calls :cmake:macro:`catkin_project` which generates
``find_package`` files; this is for other packages to use.  Each
project does a ``find_package`` of each other; in this way, packages
can build with their dependencies in the same buildspace, or already
installed on the system.  Different versions of these files are
created for use in the buildspace, and in the installation.  The
stress-test for this scheme is message generation; different language
generators must implement a specific cmake interface which is used by
``genmsg``.  See also :ref:`find_package_internals`.

.. rubric:: Python path forwarding

A special thunk (via the standard python package `pkgutil
<http://docs.python.org/library/pkgutil.html>`_ is generated and put
in ``CMAKE_BINARY_DIR/gen/py/PACKAGENAME/__init__.py``, which
extends the ``PYTHONPATH`` to include
``CMAKE_SOURCE_DIR/path/to/PACKAGENAME/src``.  This way the
buildspace PYTHONPATH need only contain ``CMAKE_BINARY_DIR/gen/py``.
Caveat: it will also need to contain ``CMAKE_BINARY_DIR/lib`` (for
compiled python bindings).  At installation time, this
thunk-__init__.py disappears and the static python library source is
installed alongside the generated message code in the same
directory.

.. rubric:: Environment generation

When cmake runs, it knows the locations of the ``CMAKE_BINARY_DIR``
and so forth, it generates an environment file (in
``CMAKE_BINARY_DIR``).  Projects may extend this environment via
:cmake:macro:`catkin_add_env_hooks`.
