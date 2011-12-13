Catkin cmake macro reference
============================

.. cmake:macro::  install_cmake_infrastructure(projectname [parameters])

   :param projectname: requires the same value as passed to cmake's ``project()``
   :param VERSION: the version in MM.NN.PP format
   :param INCLUDE_DIRS: source-relative path to C/C++ headers
   :param LIBRARIES: names of library targets 
   :param MSG_DIRS: source-relative paths to directories containing messages
   :param PYTHONPATH: source-relative paths to directory containing
      static python code.  A "thunk" will be created to this (source)
      directory from the build directory if :cmake:macro:`enable_python` is called.

   :var DONK_badonk: badonkadongk
   :outvar PROJECT_SHARE_INSTALL_PREFIX: set to
      ``CMAKE_INSTALL_PREFIX/share/${PROJECT_NAME}`` by default.  For
      use with cmake ``install()`` macro.

   Create buildspace :term:`config-mode infrastructure`.

   This macro does a lot of other stuff.  Clean me up.

   .. rubric:: Example

   ::

     install_cmake_infrastructure(c
       VERSION 0.0.1
       INCLUDE_DIRS include
       LIBRARIES c-one c-two
       MSG_DIRS msg
       )

.. cmake:macro:: enable_python(projectname)

   :param projectname: Name of project, requires same value as passed to cmake ``project()``

   Creates forwarding python :term:`pkgutil` infrastructure that "points"
   from the part of the build directory that holds :term:`generated
   code` (python) back to the source directories that hold
   :term:`static code`.


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



