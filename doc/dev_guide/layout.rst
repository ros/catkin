Pre- and Post-installation Disk Layout reference
================================================

.. highlight:: catkin-sh

.. rubric:: Source directory

This page describes the intended layout of build and install folders, in particular with respect to the Filesystem Hierarchy standard (FHS).


e.g. for project 'proj'::

  src/
    proj/
      CMakeLists.txt
      stack.xml  # explains interstack dependencies and debian-generating rules
      msg/
        ProjMsg.msg
      srv/
        ProjSrv.srv
      src/
        proj/
          __init__.py
          othercode.py
        libproj/
          CMakeLists.txt
          proj.cpp
      include/
        proj/
          proj.hpp
          otherheaders.hpp

.. rubric:: Build directory

::

  build/
    CMakeCache.txt
    cmake_install.cmake
    Makefile

    buildspace/               # the layout of that folder follows the (see install directory)
      bin/                    # just "anointed" central binaries (i.e. rosrun)

      etc/                    # environment hooks, configuration files
        catkin/
          profile.d/
            10.ros.sh         # e.g. defining the ROS_MASTER_URI
        langs/                # to determine which message generators are available
          roscpp              # contains "C++"
          rospy               # contains "Python"

      include/                # header files of generated code

      lib/                    # all compiled libraries go here
        pkgconfig/            # generated .pc files for all projects
        pythonX.Y/
          dist-packages/
            proj/             # generated Python code
              __init__.py     # generated file to relay imports into src directory of that project
        proj/                 # compiled binaries

      share/                  # all project-specific but architecture independent files
        proj/                 # one folder per package
          cmake/              # generated projConfig.cmake and projConfig-version.cmake for find_package()

      .CATKIN_WORKSPACE       # identifies folder as a catkin workspace
                              # it contains a semicolon separated list of source folders if the workspace is a buildspace
      env.sh
      setup.bash
      setup.py
      setup.sh
      setup.zsh

    CMakeFiles/


    projN/                    # the usual CMake-generated stuff
      cmake/
      CMakeFiles
      cmake_install.cmake
      Makefile
      ...


.. rubric:: Install directory

The layout of the install directory follows the `Filesystem Hierarchy Standard (FHS) <http://en.wikipedia.org/wiki/Filesystem_Hierarchy_Standard>`_.

::

  /opt/ros/groovy/            # defined by the CMAKE_INSTALL_PREFIX
                              # very similar to the buildspace folder
                              # therefore in the following only the differences are mentioned

    lib/
      pythonX.Y/
        dist-packages/
          proj/               # besides the generated Python code
                              # contains the Python source code of project

    include/                  # besides the generated header files
                              # contains all header files from the source directories

    share/
      proj/                   # further resources (i.e. icons) copied from source directory
        manifest.xml          # provide export information for rosmake and pluginlib
        action/
        msg/
          Foo.msg
          Bar.msg
        something.launch      # the rest is as the package installs it

     stacks/
       dry_stack1             # built/installed via rosmake
       dry_stack2             # built/installed via rosmake
