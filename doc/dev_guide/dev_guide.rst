Developers Guide
================


.. toctree::
   :maxdepth: 1

   underlay
   sketch
   environment
   variables
   layout
   code_generation
   maintainer
   cmake
   glossary

Installation
------------

The buildspace has *one* install target, which creates an :term:`FHS`
compliant (or as close as we can get) filesystem under
``CMAKE_INSTALL_PREFIX`` via ``make install``.  This install target
obeys ``DESTDIR`` for ease of packaging.  Projects specify what should
be installed and where in the usual cmake fashion, via the
``install()`` macro.  Resources, assets, launchfiles, etc get
installed using a set of destination variables (:ref:`variables`).


Build Parameters
----------------

* The standard set of cmake variables, especially
  ``CMAKE_INSTALL_PREFIX``, ``CMAKE_BUILD_TYPE``,
  ``CMAKE_TOOLCHAIN_FILE`` etc.

* The language projects that are found in the buildspace.

* Assorted others as needed by individual stacks/packages, i.e. to
  enable/disable certain features or dependencies.
