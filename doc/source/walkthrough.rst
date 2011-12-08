Quickstart
==========

Pick a temp directory::

  % cd /tmp

Get the rosinstall file::

  % wget https://raw.github.com/willowgarage/catkin/master/test/test.rosinstall

Rosinstall it.  You don't want it built, use ``-n``, like this::

  % rosinstall -n src test.rosinstall

  [lots of output ending in...]

  rosinstall update complete.

  Now, type 'source /tmp/src/setup.bash' to set up your environment.
  Add that to the bottom of your ~/.bashrc to set it up every time.

  If you are not using bash please see http://www.ros.org/wiki/rosinstall/NonBashShells 

There will be lots of output.  Currently you need a toplevel
``CMakeLists.txt``, make a symlink to ``catkin/toplevel.cmake``::

  % cd src
  % ln -s catkin/toplevel.cmake CMakeLists.txt
  % ls 
  catkin/          common_msgs/  genmsg/	genpybindings/	ros_comm/     setup.bash  setup.zsh
  CMakeLists.txt@  gencpp/       genpy/	        ros/		roscpp_core/  setup.sh	  std_msgs/

Now do the typical cmake thing::

  % mkdir build
  % cd build
  % cmake ..
  -- The C compiler identification is GNU
  -- The CXX compiler identification is GNU

    ...

  -- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  -- ~v^V^v~   traversing stacks/projects in topological order   ~v^V^v~
  -- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    ...

  -- Configuring done
  -- Generating done
  -- Build files have been written to: /tmp/src/build

And make::

  % make
  Scanning dependencies of target cpp_common
  [  0%] Building CXX object roscpp_core/cpp_common/CMakeFiles/cpp_common.dir/src/debug.cpp.o
  Linking CXX shared library ../../lib/libcpp_common.so

    ...

  [ 96%] Generating Python __init__.py for nav_msgs
  [ 97%] Built target nav_msgs_genpy
  Scanning dependencies of target nav_msgs_genpybindings
  [100%] Built target nav_msgs_genpybindings
  

  


