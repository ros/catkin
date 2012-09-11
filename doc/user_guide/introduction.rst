Introduction
============

.. highlight:: catkin-sh

Catkin helps you with building ROS packages. It does so by leveraging
the cmake tool that has for years been the most used C++ build
framework. The focus of catkin is C++ and python based packages.

Users experienced with cmake should feel right at home with catkin.

What does Catkin provide?
-------------------------

The ROS ecosystem has a philosophy of federated development. Plenty of
teams around the world develop their own modular ROS
packages. Installing, packaging and deploying those packages is a
common chore, and the catkin macros make all these tasks easier, and
provide a standard so that it becomes easier to use code by other teams.

By encouraging best practice standards, ROS packages should in the
future become more portable and maintainable. Catkin encourages to
adhere to

* The Filesystem Hierarchy Standard (FHS)
* The CMake standards and best practices

Catkin is also extensible, such as allowing automatic inclusion of
genmsg, the framework of defining ROS messages with references to
messages in other ROS packages.

Finally Catkin provides a way to compile C++ sources from several
independent packages the fastest way possible, to improve the
development cycle.


If you have used rosmake before...
----------------------------------

The workflow and commands of catkin and rosmake are different.

The documentation provides both a migration guide for existing ROS
stacks and packages as well as a comparison of the workflow steps.

