Developers Guide
================

This section of the documentation is provided for developers of
catkin itself and advanced users.

.. toctree::
   :maxdepth: 1

   sketch
   underlay
   environment
   variables
   layout
   code_generation
   generated_cmake_api
   maintainer
   glossary

Installation
------------

The buildspace has *one* install target, which creates an :term:`FHS`
compliant (or as close as we can get) filesystem under
``CMAKE_INSTALL_PREFIX`` via ``make install``.  This install target
obeys ``DESTDIR`` for ease of packaging.  Packages specify what
should be installed and where in the usual CMake fashion, via the
``install()`` macro.  Resources, assets, launchfiles, etc get
installed using a set of destination variables (:ref:`variables`).


Build Parameters
----------------

* The standard set of CMake variables, especially
   ``CMAKE_INSTALL_PREFIX``, ``CMAKE_BUILD_TYPE``,
   ``CMAKE_TOOLCHAIN_FILE`` etc.

* Assorted others as needed by individual packages, i.e. to
   enable/disable certain features or dependencies.
