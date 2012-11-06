.. _setup_dot_py_handling:

Handling of ``setup.py``
------------------------

If your ROS package contains python scripts to install, you need a
setup.py file to manage the installation process. This is standard for
Python, and catkin will use such a file if provided, and if you include the line::

  catkin_python_setup()

in the CMakeLists.txt of your project.

The setup.py file uses Python ``distutils`` or ``setuputils`` to describe the Python content of the stack.

It is used during build to symlink all scripts listed in ``scripts`` to a folder in devel space where they can be found and executed.

Furthermore it is used to install the Python packages to the correct path under ``CMAKE_INSTALL_PREFIX``.
