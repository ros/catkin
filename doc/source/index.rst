Catkin
======

`Catkin <http://en.wikipedia.org/wiki/Catkin>`_ is a collection of
cmake macros and associated code used to build some parts of ROS as of
the *fuerte* release.


Terminology
-----------

Stack:    unit of installation.

Package: generally something "smaller" than a stack... but a package
can be a stack.


Design sketch
-------------

* There is only one invocation of cmake (and make) to build a source
  directory containing N stacks.  

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
  poorest taste.  Exception: sphinx-generated Makefiles used to
  generate documentation.


Build Parameters
----------------

* The language projects that are found in the buildspace.
* The standard set of cmake variables, especially
  ``CMAKE_INSTALL_PREFIX``, ``CMAKE_BUILD_TYPE``, etc.
* Assorted others as needed by individual stacks/packages, i.e. to
  enable/disable certain features or dependencies.


Installation
------------

The buildspace has *one* install target, which creates an FHS
compliant (or as close as we can get) filesystem under
``CMAKE_INSTALL_PREFIX`` via ``make install``.  This install target
obeys ``DESTDIR`` for ease of packaging.  Projects specify what should
be installed and where in the usual cmake fashion, via the
``install()`` macro.


Main trickery
-------------

* During the cmake run the main source directory is scanned for stacks
  to build.  Actually cmake doesnt care if it is a stack or a package;
  it cares that the directory has something there that indicates that
  it is buildable, and what its dependencies are.  Catkin
  topologically sorts this and reads the files in order.

* Each project calls a macro that generates ``find_package`` files;
  this is for other packages to use.  Each project does a
  ``find_package`` of each other; in this way, packages can build with
  their dependencies in the same buildspace, or already installed on
  the system.  Different versions of these files are created for use
  in the buildspace, and in the installation.

* A special thunk (via the standard python package `pkgutil
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

* All C++ headers are generated into ``CMAKE_BINARY_DIR/gen/cpp``;
  only this directory need be in one's include path.

* When cmake runs, it knows the locations of the ``CMAKE_BINARY_DIR``
  and so forth, it generates an environment file (in
  ``CMAKE_BINARY_DIR``)


