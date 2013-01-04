Catkin workspace layout
=======================

The directory containing the catkin packages must contain a symlink::

   CMakeLists.txt -> <path/to/catkin>/cmake/toplevel.cmake

which triggers catkin's topological sort via inspection of all
``package.xml`` files in that directory.
