Getting started with Catkin
===========================

.. highlight:: catkin-sh

Creating a workspace
--------------------

This page gives step-by-step instructions on setting up a workspace to
develop ROS packages on top of ROS groovy.

To create a workspace for ROS `Groovy <http://ros.org/wiki/groovy>`_,
you first need to install ROS Groovy.

Once you have done so, choose a location on your disk to use as a
workspace for the ROS packages you will use. For this guide we will
choose `~/groovy_overlay`::

   $ mkdir groovy_overlay
   $ cd groovy_overlay
   $ mkdir src
   $ cd src
   $ catkin_init_workspace

The last command creates a CMakeLists.txt link in `src`.
We now have an empty catkin workspace. As usual with CMake, we create
a build folder where all build output will go to (e.g. compiled files)::

   $ cd ~/groovy_overlay
   $ mkdir build

Next, even though we have not added any package to our workspace, we
can still check and configure our workspace::

   $ cd build
   $ cmake ../src

This generates several folders and files in our build folder. 

.. note:: You need to run this command every time you add or remove packages from your workspace, change manifests or CMakeLists.txt.

Important for you are the `Makefile` and the `buildspace`.

The `Makefile` allows you to run the command `make` in the build
folder, which will build all the projects in src (currently None).
The `buildspace` folder contains files you can use to work with the
build results as if they had already been installed.

.. note:: This is an important feature of catkin. Without catkin, you would need to run `make install` after each time you run `make`, and manage another location with the install results.

To use the `buildspace`, your environment needs to be set up, that
means several variables like PATH, PYTHONPATH, etc. need to also point
to the folder `~/groovy_overlay/build/buildspace`. This is achieved by
files that the CMake command generated in `buildspace`. You can set up
your environment by calling::

   $ source ~/groovy_overlay/build/buildspace/setup.sh

You can add this command to your .bashrc if you want it to be executed
every time you open a shell.

Once you have added projects to your workspace, you can build
the projects like this::

   $ cd ~/groovy_overlay/build
   $ make -j8

The `-j8` option tells make to use 8 jobs, making full use of 8 CPU cores. If you have more or less, adjust the parameter.

Installing
----------

With Catkin, you do not need to properly install files on Ubuntu, as
you can use the buildspace more conveniently. However expert users or
users on other operating systems may need to properly install packages.

As with standard CMake, you do so by specifying an install path in CMake::

   $ cd ~/groovy_overlay/build
   $ cmake ../src -DCMAKE_PREFIX_PATH=~/groovy_overlay/install
   $ make -j8
   $ make install

You can use anything as an install path, but for system-wide
installation, you will neet root privileges. Do not use `/opt/ros` as
install targets for Ubuntu systems to avoid conflicts with installable
Debian packages.

Uninstalling
------------

Unless you ran `make install` with a system prefix, you can always
cleanly remove all packages by deleting the `build` folder.

By default, the only safe way to uninstall individual packages is to
delete the `buildspace` folder, delete the sources of the packages,
and run make again. You can also remove the `build` folder and go
through the steps above again.

Adavanced users uninstalling often should investigate advanced tools
for instllation management like GNU stow.
