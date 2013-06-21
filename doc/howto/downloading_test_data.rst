.. _downloading_test_data:

Downloading test data
---------------------

When unit tests require large data files, it is better to download
them from the web than include them in your source repository.

Each of these files declares a target name.  Running ``"make tests"``
or ``"make run_tests"`` will automatically download them all.
Individual test targets do *not* depend on the ``tests`` target, but
you can use it to download them before running one::

  $ make tests run_tests_your_package

Catkin provides a convenient command, used like this::

  catkin_download_test_data(
    ${PROJECT_NAME}_32e.pcap
    http://pr.willowgarage.com/data/velodyne/32e.pcap
    MD5 e41d02aac34f0967c03a5597e1d554a9)

The first parameter is the target name, which should normally include
your project name to avoid conflicts with other packages.

The second parameter is the URL to read.  If you release your package
to the ROS build farm, make sure this URL is available reliably.
Otherwise, tests will fail randomly, annoying everyone.  Contact
``ros-release@code.ros.org`` if you need to host some data near the
build servers.

The MD5 argument is a good way to avoid testing with corrupted data.

The default destination for the file downloaded above is within your
package's build directory.  The file name is the base name of the URL.
You can use the DESTINATION argument to put it somewhere else.

For example, sometimes a rostest_ script wants to use ``$(find
your_package)`` to access the test data.  Here is how to put the file
in devel-space, where roslaunch_ can resolve it::

  catkin_download_test_data(
    ${PROJECT_NAME}_32e.pcap
    http://pr.willowgarage.com/data/velodyne/32e.pcap
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    MD5 e41d02aac34f0967c03a5597e1d554a9)

Then, the test script can pass it as a parameter, like this::

  <node pkg="your_package" type="your_node" name="your_node">
    <param name="data" value="$(find your_package)/tests/32e.pcap"/>
  </node>


.. _roslaunch: http://ros.org/wiki/roslaunch/XML#substitution_args
.. _rostest: http://ros.org/wiki/rostest
