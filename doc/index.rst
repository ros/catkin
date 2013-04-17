Catkin
======

`Catkin <http://en.wikipedia.org/wiki/Catkin>`_ is a collection of
CMake macros and associated code used to build packages used in
`ROS <http://www.ros.org>`_.

It has been introduced as part of the
`Fuerte <http://ros.org/wiki/fuerte>`_ release where it was used for
a small set of base packages. For
`Groovy <http://ros.org/wiki/groovy>`_ it as been significantly
modified and is used by a large set of packages.

This version of the documentation is specific to the
`Groovy <http://ros.org/wiki/groovy>`_ release (or later).

Key benefits of catkin:

* Standard compliance (FHS, Cmake)
* Convenient and fast configuration of multiple projects
* Fast build of multiple projects
* Support for C++ and python projects
* Build-space can be used like an install-space (since groovy)

Installation
------------

Prerequisites:

catkin has the following dependencies:

* CMake - A cross-platform, open-source build system.
* Python - Python is a general-purpose, interpreted high-level programming language.
* catkin_pkg - A Python runtime library for catkin.
* empy - A Python template library.
* nose - A Python testing framework.
* GTest - A cpp unittest framework from Google.

You can resolve these dependencies on Ubuntu with this command::

  sudo apt-get install cmake python-catkin-pkg python-empy python-nose libgtest-dev

If you are not on Ubuntu you can also install catkin_pkg from PyPi via pip.

catkin itself is distributed as binary debian package for Ubuntu, it can be installed via::

  sudo apt-get install ros-groovy-catkin

However installing ROS will also install catkin.

Code & tickets
--------------

+--------------+-------------------------------------+
| Catkin       | http://github.com/ros/catkin        |
+--------------+-------------------------------------+
| Issues       | http://github.com/ros/catkin/issues |
+--------------+-------------------------------------+
| A list of    | https://github.com/ros/rosdistro/   |
| catkinized   | releases/groovy.yaml                |
| ROS packages |                                     |
+--------------+-------------------------------------+


Contents
--------

.. toctree::
   :maxdepth: 2

   user_guide/user_guide
   howto/index
   adv_user_guide/adv_user_guide
   dev_guide/dev_guide
