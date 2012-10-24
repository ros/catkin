Migrating from rosbuild to catkin
=================================

From Box Turtle to Fuerte, the ROS build system was `rosbuild
<http://ros.org/wiki/rosbuild>`_. In Fuerte, `catkin
<http://ros.org/wiki/catkin>`_ has been introduced for core ROS
stacks.

The goal is to deprecate rosbuild in the future, and encourage
users to migrate their code from rosbuild to catkin. We especially
encourage maintainers of released ROS packages to migrate early, because
the build farm that generates binary Debian packages from released ROS
stacks will eventually be decommissioned in the future.

Since catkinized packages cannot depend on rosbuild packages, the
migration has to start with core stacks and move on along the
dependency chain. This is the motivation for the wet/dry metaphor
along a rising tide.

Quick start guide
.................

At a high level, the key steps to convert a ROS stack from rosbuild to
catkin packages are:

1. Create a catkin workspace to hold your migrated projects
2. Move any project to this catkin workspace once all its dependencies are catkinized
3. Convert your stack into a metapackage, unless it was a unary rosbuild stack (Unary rosbuild stacks are stacks with just one package, those will become one catkin package)

 a. create a catkin package in your old stack folder and name it like the stack, this is going to be the metapackage (as an example, see ros_comm/ros_comm)
 b. add all other packages of the old stack as run_depend tags of the metapackage in the package.xml
 c. remove the stack.xml, the information from it moves into the package.xml of the metapackage
 d. What previously was your 'stack' folder should not contain only packages and at most one metapackage.

4. For each folder containing a ``manifest.xml`` file, do the following steps.
   You can use the utility catkinize_manifest_xml_to_package_xml.py from
   `catkinize <https://github.com/ros-infrastructure/catkinize>`_ to partially
   automate this step.

 a. rename the ``manifest.xml`` to ``package.xml``
 b. add a name tag with the name of the package, which should also be the folder name
 c. If missing, create a CMakeLists.txt file containing a ``catkin_package()`` invocation

5. In each ``CMakeLists.txt``, do the following steps. You can use the script
   catkinize_cmakelists.py from `catkinize
   <https://github.com/ros-infrastructure/catkinize>`_ to do some of the work.

 a. If rosbuild macros were used, switch from rosbuild macros to the underlying CMake commands
 b. Declare how your targets (c++ binaries) shall be installed

Detailed migration guide
........................

rosbuld wrapped several CMake macros with macros named
``rosbuild_...``. Catkin attempts to not wrap commands,
so you will be using the standard cmake macros, with only
a few added catkin macros if necessary.

- Macro migration:

 - Build macros:

  - ``rosbuild_init()`` -> remove this
  - ``rosbuild_add_library(...)`` -> ``add_library(...)``
  - ``rosbuild_add_executable(...)`` -> ``add_executable(...)``
  - ``rosbuild_add_compile_flags(target added_flags)`` -> ``set_target_properties(target PROPERTIES COMPILE_FLAGS new_flags)``
  - ``rosbuild_remove_compile_flags(target removed_flags)`` -> ``set_target_properties(target PROPERTIES COMPILE_FLAGS new_flags)``
  - ``rosbuild_add_link_flags(target added_flags)`` -> ``set_target_properties(target PROPERTIES LINK_FLAGS new_flags)``
  - ``rosbuild_remove_link_flags(target removed_flags)`` -> ``set_target_properties(target PROPERTIES LINK_FLAGS new_flags)``
  - ``rosbuild_add_boost_directories(); rosbuild_link_boost(target components)`` ->
   - once in CMakeLists.txt with all components::

     ``find_package(Boost REQUIRED COMPONENTS components)``
     ``include_directories(${Boost_INCLUDE_DIRS})``
   - for each target::

     ``target_link_libraries(target ${Boost_LIBRARIES})``
  - ``rosbuild_add_openmp_flags()`` -> ``find_package(OpenMP)``, then do other stuff
  - ``rosbuild_invoke_rospack()`` -> don't do this, use find_package to locate ROS packages as well
  - ``rosbuild_find_ros_package()`` -> don't do this, use find_package to locate ROS packages as well
  - ``rosbuild_find_ros_stack()`` -> don't do this, stacks are obsolete, use find_package to find packages instead
  - ``rosbuild_check_for_sse()`` -> look around online and find an example of how to find SSE
  - ``rosbuild_include(package module)`` -> ``include(module)`` (might require some initial work to find the path to the module)
  - ``rosbuild_add_lisp_executable()`` -> no support for this currently

  - ``rosbuild_add_swigpy_library(target lib src1 src2)`` ->

 - Test macros:

  - ``rosbuild_add_gtest(...)`` -> ``catkin_add_gtest(...)``
  - rosbuild_add_gtest_labeled -> don't do this, use catkin_add_gtest
  - rosbuild_add_gtest_future -> don't do this, just comment out tests
  - rosbuild_add_gtest_build_flags -> TODO
  - ``rosbuild_add_pyunit`` -> ``catkin_add_nosetests(...)``
  - rosbuild_add_pyunit_labeled -> ``catkin_add_nosetests``
  - rosbuild_add_pyunit_future -> ``catkin_add_nosetests``
  - rosbuild_add_rostest -> TODO
  - rosbuild_add_rostest_labeled -> TODO
  - rosbuild_add_rostest_future -> TODO
  - rosbuild_add_roslaunch_check -> TODO
  - rosbuild_declare_test -> TODO
  - rosbuild_count_cores -> TODO
  - rosbuild_check_for_display -> TODO
  - rosbuild_check_for_vm -> TODO

- Message / service macros

 - ``rosbuild_add_generated_msgs(...)`` -> ``add_message_files(DIRECTORY msg FILES ...)``
 - ``rosbuild_add_generated_srvs`` -> ``add_service_files(DIRECTORY srv FILES ...)``
 - ``rosbuild_genmsg()`` ->  , ``generate_messages()``
 - ``rosbuild_gensrv`` -> ``generate_messages()``

- Version macros

 - ``rosbuild_get_stack_version`` -> obsolete
 - ``rosbuild_get_package_version`` -> obsolete

- Data macros

 - ``rosbuild_download_data(url filename [md5sum])`` -> TODO
 - ``rosbuild_download_test_data`` -> ``download_test_data``
 - ``rosbuild_untar_file`` -> TODO

- Special targets

 - ``rosbuild_premsgsrvgen`` -> TODO
 - ``rosbuild_precompile`` -> TODO
 - ``rosbuild_make_distribution`` -> TODO
