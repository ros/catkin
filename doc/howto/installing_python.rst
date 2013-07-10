.. _installing_python_1:

Installing Python scripts and modules
-------------------------------------

Even if ``your_package`` only contains Python code, it still needs a
catkin ``CMakeLists.txt`` to install executable scripts and to export
modules so they can be imported in other ROS packages.


Scripts
:::::::

ROS executables are installed in a per-package directory, not the
distributions's global ``bin/`` directory.  They are accessible to
rosrun_ and roslaunch_, without cluttering up the shell's ``$PATH``,
and their names only need to be unique within each package.  There are
only a few core ROS commands like ``rosrun`` and ``roslaunch`` that
install in the global ``bin/`` directory.

Standard ROS practice is to place all your executable Python programs
in a ``scripts/`` subdirectory.  To keep the user API clean,
executable script names generally do not include a ``.py`` suffix.
Your ``CMakeLists.txt`` can conveniently install all the scripts in
that directory::

  install(DIRECTORY scripts/
          DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
          PATTERN ".svn" EXCLUDE)

The ``PATTERN ".svn" EXCLUDE`` is only needed if you use a Subversion_
repository.  For other types of repositories, it can be omitted.

Other directory names are allowed.  If you mixed the scripts with
other files, you should install them one by one::

  install(PROGRAMS scripts/your_node1 scripts/your_node2
          DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

Another good practice is to keep executable scripts very short,
placing most of the code in a module which the script imports and then
invokes::

  #! /usr/bin/env python
  import your_package.main
  if __name__ == '__main__':
      your_package.main()


Modules
:::::::

Standard ROS practice is to place Python modules under the
``src/your_package`` subdirectory, making the top-level module name
the same as your package.  Python requires that directory to have an
``__init__.py`` file, too.

  **Note:** with rosbuild, it was possible to place Python modules
  directly within ``src/``, but catkin does not allow that.  If you
  need to define a module named ``your_package``, place its code in
  ``src/your_package/__init__.py`` or else import its public symbols
  there.

Catkin installs Python packages using a variant of the standard Python
``setup.py`` script.  Assuming your modules use the standard ROS
layout, it looks like this::

  ## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

  from distutils.core import setup
  from catkin_pkg.python_setup import generate_distutils_setup
  
  # fetch values from package.xml
  setup_args = generate_distutils_setup(
      packages=['your_package'],
      package_dir={'': 'src'})
  
  setup(**setup_args)

This ``setup.py`` is only for use with catkin. Remember *not* to
invoke it yourself.

Put that script in the top directory of your package, and add this to
your ``CMakeLists.txt``::

  catkin_python_setup()

That takes care of installing your Python modules.  Never use it to
install executable scripts, use the ``install()`` command shown above.

.. _roslaunch: http://ros.org/wiki/roslaunch
.. _rosrun: http://ros.org/wiki/rosrun
.. _Subversion: http://subversion.apache.org/
