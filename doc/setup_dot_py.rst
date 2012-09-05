.. _setup_dot_py_handling:

Handling of ``setup.py``
------------------------

The setup.py file uses Python ``distutils`` or ``setuputils`` to describe the Python content of the stack.

It is used during build to symlink all scripts listed in ``scripts`` to a folder in buildspace where they can be found and executed.

Furthermore it is used to install the Python packages to the correct path under ``CMAKE_INSTALL_PREFIX``.
