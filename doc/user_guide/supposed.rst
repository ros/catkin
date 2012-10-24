Catkin workspace layout
=======================

A Catkin workspace is a folder with a CMakeLists.txt file. It acts
like one huge cmake project containing any cmake project that you 
add as subfolder as a subproject.

.. contents::


``src`` (the Workspace folder)
------------------------------

You can place your workspace folder wherever you want and name it
what you want. One standard layout for ROS is to have a folder named
like the distro you use this workspace for, and in that folder a
folder named ``src``, to use as workspace folder.

In the ``src`` folder, you can use
`rosws <http://www.ros.org/doc/api/rosinstall/html/>`_ to download
and update catkin packages from multiple sources.

This directory must contain a symlink::

   CMakeLists.txt -> path/to/catkin/cmake/toplevel.cmake

which triggers catkin's topological sort via inspection of all
``package.xml`` files in the workspace.
