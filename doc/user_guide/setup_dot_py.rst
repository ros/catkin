.. _setup_dot_py_handling:

Handling of ``setup.py``
------------------------

If your ROS package contains python scripts to install, you would need to define
the installation process and a way to make the scripts accessible before or without
installation.

The python ecosystem defines installation standards in the
``distutils`` or ``setuputils`` libraries. With those libraries,
packages define the installation files in a file called ``setup.py``
in the project root. The setup.py file uses Python to describe the
Python content of the stack.

Catkin allows you to specify the installation of
your python files in this setup.py and reuse the information in your
CMakeLists.txt.

You can do so by including the line::

  catkin_python_setup()

in the CMakeLists.txt of your project.

This creates relays for all scripts listed in ``scripts`` to a folder in devel space where they can be found and executed.

Furthermore it is used to install the Python packages to the correct path under ``CMAKE_INSTALL_PREFIX``.
