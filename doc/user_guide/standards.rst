CMake coding standards
======================

In order to make your ROS packages most maintainable and avoid several
frequent errors, follow the standards below.

**Call catkin_project early**

The following line should always appear like this in this order
without other commands in between::

  cmake_minimum_required(VERSION 2.8.3)
  project(myproject)
  find_package(catkin REQUIRED <COMPONENTS ...>)
  catkin_project(<...>)


**Use ${PROJECT_NAME} wherever possible**

Use ``${PROJECT_NAME}`` for variables, targets and labels instead of
repeating the project name manually or using fixed names.

You can use ${PROJECT_NAME} as prefix for variable names as well, as shown in examples below.

After you defined your project name like this::

   project(myproject)

dont do this::

  catkin_add_gtest(test ...)
  add_executable(myproject ...)
  set(use_feature 42 CACHED STRING "description")
  option(use_feature "on or off" OFF)
  macro(xyz) ...

do this instead::

  catkin_add_gtest(${PROJECT_NAME}_test ...)
  add_executable(${PROJECT_NAME} ...)
  set(${PROJECT_NAME}_use_feature 42 CACHED STRING "description")
  option(${PROJECT_NAME}_use_feature "on or off" OFF)
  macro(${PROJECT_NAME}_xyz) ...

This will avoid conflicts between packages and errors due to copy&paste.

**find_package(... REQUIRED)**

Use ``REQUIRED`` on all calls to ``find_package`` if they aren't
actually optional (i.e. you're not going to check ``thing_FOUND``
and enable/disable features).


**Keep lists sorted**

Whenever using a list of items (i.e. in find_package(COMPONENTS ...)
or files which should be build or installed) keep them alphabetically
sorted.  This improves readability when looking for specific items.
(There are exceptions which require a specific custom order like the
list of projects inside a stack).

**Lowercase keywords**

Keywords like ``if``, ``for`` etc. are all lowercase.


**Closing keyword should have empty parenthesis**

The closing keywords like ``endif()`` and ``endforeach()`` should not repeat the condition of the opening keyword.
