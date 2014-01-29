Updating a catkin for Indigo
============================

If you catkin package is working in Groovy/Hydro it should be easy to
get it working in Indigo too.

Removed deprected functions
---------------------------

The following CMake functions and macros which have were deprecated
before have been removed from catkin in Indigo. Please replace them
with there replacement:

* ``add_gtest(..)`` => ``catkin_add_gtest(..)``
* ``add_nosetests(..)`` => ``catkin_add_nosetests(..)``
* ``download_test_data(url md5)`` => ``catkin_download_test_data(target url MD5 md5)``
* ``parse_arguments(..)`` => ``cmake_parse_arguments(..)``

Removed Eigen CMake config file
-------------------------------

The previously provided CMake config file for Eigen has been removed.
Instead you might want to use the CMake module provided by the
`cmake_modules <http://wiki.ros.org/cmake_modules>`_ package.
