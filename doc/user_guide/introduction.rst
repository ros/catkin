Introduction
============

.. highlight:: catkin-sh

Catkin helps building ROS packages by leveraging
the cmake tool that has for years been the most used C++ build
framework. The focus of catkin is C++ and python based packages.

Users experienced with cmake should feel right at home with catkin.

What does Catkin provide?
-------------------------

The ROS ecosystem has a philosophy of federated development. Many
teams around the world develop their own modular ROS
packages. Building, installing, packaging and deploying those packages
is a common chore, and the catkin macros make all these tasks easier,
and provide a standard so that it becomes easier to use code by other
teams.

By encouraging best practice standards, ROS packages should in the
future become more portable and maintainable. Catkin encourages to
adhere to

* The Filesystem Hierarchy Standard (FHS)
* The CMake standards and best practices

Catkin is also extensible, such as allowing automatic inclusion of
genmsg, the framework of defining ROS messages with references to
messages in other ROS packages.

Finally Catkin provides a way to compile C++ sources from several
independent packages faster than by using only cmake on each project
individually.


If you have used rosmake before using catkin
--------------------------------------------

The workflow and commands of catkin and rosmake are different.

The documentation provides both a migration guide for existing ROS
stacks and packages as well as a comparison of the workflow steps.

.. toctree::
   :maxdepth: 1

   rosbuild_migration

If you have used cmake before using catkin
------------------------------------------

To get all catkin benefits you will need to learn a few catkin macros
on top of cmake macros.

Also you need to be aware of some restrictions catkin imposes on
CMakeLists.txt files.

.. toctree::
   :maxdepth: 1

   standards
