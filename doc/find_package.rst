.. _find_package_internals:

Internals of the generated find_package() infrastructure
--------------------------------------------------------

CMake's ``find_package`` is the preferred method for packages to
communicate to CMake (and thereby to catkin) the libraries, include
directories and such that packages should use.

There are a couple of modes of operation of find_package (see the
CMake documentation), "module" mode and "config" mode.  "module" mode
is the one that uses cmake scripts named ``Find****.cmake``.  "config"
mode is the preferred mode, and it works differently.

One reason we find the 'config mode' superior is that is supports
multiple simultaneous installed versions of packages.

For a package T, 'config mode' consists of two cmake files, both of
which are installed somewhere on cmake's ``CMAKE_PREFIX_PATH``.  The
first is 't-config-version.cmake'.  We find the most succinct form to
be like this::

  set(PACKAGE_VERSION_EXACT False)
  set(PACKAGE_VERSION_COMPATIBLE False)
  if("${PACKAGE_FIND_VERSION}" STREQUAL "")
    set(PACKAGE_VERSION_COMPATIBLE TRUE)
    return()
  endif()

  if("${PACKAGE_FIND_VERSION}" VERSION_EQUAL "9.9.9")
    set(PACKAGE_VERSION_EXACT True)
    set(PACKAGE_VERSION_COMPATIBLE True)
  endif()

  if("${PACKAGE_FIND_VERSION}" VERSION_LESS "9.9.9")
    set(PACKAGE_VERSION_COMPATIBLE True)
  endif()

where `9.9.9` is replaced by the numeric version of the package.  The
second file, ``t-config.cmake``, tells the client what the assorted
includes/libs are for the package by setting variables
``t_INCLUDE_DIRS``, ``t_LIBRARIES`` and so forth.  The user passes
these values to cmake's ``include_directories()`` and
``target_link_libraries()``.


Finding ROS by component (recommended method)
---------------------------------------------

If you want to specify a dependency on several ROS components
simultaneously, use
``find_package(ROS [XX.YY] REQUIRED COMPONENTS comp1 comp2)``, e.g.::

  find_package(ROS REQUIRED COMPONENTS
               cpp_common rostime roscpp_traits
               roscpp_serialization sensor_msgs)
  include_directories(${ROS_INCLUDE_DIRS})

  add_executable(myexec ...)
  target_link_libraries(${ROS_LIBRARIES})

See the cmake documentation for ``find_package()`` for more details.
Your ``CMAKE_PREFIX_PATH`` will need to point to a ROS installation.
The version string is as yet to be determined; "electric", for
instance, is not a valid version.  We will have to establish some
mapping between ROS versions and a numeric version of only numbers and
dots.






