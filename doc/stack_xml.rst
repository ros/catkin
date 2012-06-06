.. _stack_xml:

Stack manifest XML tags reference
=================================

NOTE: Stack manifests always have the filename ``stack.xml``.
This document describes catkin-ized stacks.

<stack>
-------

The ``<stack>`` tag is the unique top-level tag in a stack manifest.

Required Tags
-------------

The required set of tags in a ``stack.xml`` file provides basic metadata
about your stack, including what it is, who wrote it, and who can use it.

 * :ref:`\<name\> <stack_name_tag>`
 * :ref:`\<version\> <stack_version_tag>`
 * :ref:`\<description\> <stack_description_tag>`
 * :ref:`\<author\> <stack_author_tag>`
 * :ref:`\<maintainer\> <stack_maintainer_tag>`
 * :ref:`\<license\> <stack_license_tag>`
 * :ref:`\<copyright\> <stack_copyright_tag>`

Optional Tags
-------------

The most common optional tags are ``<build_depends>``, ``<depends>`` and
``<url>``. We strongly recommend the use of the ``<url>`` tag to point users
to a website where they can find out more information about your stack.
The website is most commonly a wiki page on ROS.org so that users can easily
edit and update information about your stack.

 * :ref:`\<url\> <stack_url_tag>`
 * :ref:`\<review\> <stack_review_tag>`
 * :ref:`\<build_depends\> <stack_build_depends_tag>`
 * :ref:`\<depends\> <stack_depends_tag>`
 * :ref:`\<build_type\> <stack_build_type_tag>`
 * :ref:`\<message_generator\> <stack_message_generator_tag>`

Example
-------

::

  <stack>
    <name>ros_comm</name>
    <version>0.1.2</version>
    <description brief="ROS communications-related libraries and tools">ROS communications-related packages, including core client libraries (roscpp, rospy, roslisp) and graph introspection tools (rostopic, rosnode, rosservice, rosparam).</description>
    <author>Troy Straszheim</author>
    <author>Morten Kjaergaard</author>
    <author email="kwc@willowgarage.com">Ken Conley</author>
    <maintainer email="straszheim@willowgarage.com">Troy Straszheim</maintainer>
    <license>BSD</license>
    <copyright>willowgarage</copyright>

    <url>http://www.ros.org</url>
    <review status="Doc reviewed" notes="2009/6/10"/>

    <build_depends>genmsg</build_depends>
    <build_depends>libboost-thread-dev</build_depends>
    <build_depends>libboost-date-time-dev</build_depends>

    <depends>libboost-thread</depends>
    <depends>libboost-date-time</depends>
  </stack>


.. _stack_name_tag:

<name>
------

Text
''''

The name of the stack. Should correspond to the name of the directory that the code is checked out in, if this directory is being traversed by catkin.


.. _stack_version_tag:

<version>
---------

Text
''''

The version number of the stack. Major.Minor.Patchlevel version.
Numeric only. Used by deb packaging utilities.

Attributes
''''''''''

 ``abi="1.2.1"`` *(optional)*
  An optional ABI version number if it is different from the version number.

Example
'''''''

::

  <version>1.2.3</version>


.. _stack_description_tag:

<description>
-------------

Text
''''

Description of the stack. It may be multi-line and include XHTML.

Attributes
''''''''''

 ``brief="brief text"`` *(optional)*
  One-line summary of your stack. Useful for UI displays where the stack name isn't sufficiently descriptive.

Example
'''''''

::

  <description brief="ROS for Python">
    Python implementation of the ROS master/node APIs and client library.
  </description>


.. _stack_author_tag:

<author> (multiple)
-------------------

Text
''''

The name of the person who authored the stack.

Attributes
''''''''''

 ``email="name@domain.tld"`` *(optional)*
  Email address of author.

Example
'''''''

::

    <author email="aphacker@willowgarage.com">Alyssa P. Hacker</author>


.. _stack_maintainer_tag:

<maintainer> (multiple)
-----------------------

Text
''''

The name of the person maintaining the stack.

Attributes
''''''''''

 ``email="name@domain.tld"`` *(optional)*
  Email address of maintainer.

Example
'''''''

::

    <maintainer email="aphacker@willowgarage.com">Alyssa P. Hacker</maintainer>


.. _stack_license_tag:

<license>
---------

Text
''''

Name of license for this package, e.g. BSD, GPL, LGPL. In order to
assist machine readability, only include the license name in this
tag. For any explanatory text about licensing caveats, please use the
``<description>`` tag.

Most common open-source licenses are described on the `OSI website <http://www.opensource.org/licenses/alphabetical>`_.

Commonly used license strings:

 - Apache 2.0
 - BSD
 - Boost Software License
 - GPLv2
 - GPLv3
 - LGPLv2.1
 - LGPLv3
 - MIT 
 - Mozilla Public License Version 1.1
 - ZLib
 - wxWindows

Example
'''''''

::

    <license>BSD,GPLv3</license>


.. _stack_copyright_tag:

<copyright>
-----------

Text
''''

Copyright information which is integrated into Debian packages.

Example
'''''''

::

    <copyright>Copyright (c) 2012, Willow Garage, Inc.</copyright>


.. _stack_url_tag:

<url>
-----

Text
''''

Website for the stack. This is important for guiding users to online documentation.

Example
'''''''

::

    <url>http://ros.org/wiki/navigation</url>


.. _stack_review_tag:

<review>
--------

Status of the stack in the review process (Design, API, and Code
review). `QAProcess <http://ros.org/wiki/QAProcess>`_.  Stack that
have not yet been reviewed should be marked as "experimental".

Attributes
''''''''''

 ``status="status"``
   See `list of valid review statuses <http://ros.org/wiki/Review Status>`_.
 ``notes="notes on review status"`` *(optional)*
   Notes on review status, such as date of last review.

Example
'''''''

::

    <review status="experimental" notes="reviewed on 3/14/09" />


.. _stack_depends_tag:

<depends> (multiple)
--------------------

Declares a ROS dep key that this stack depends on at run-time.
Used to determine dependency ordering of current workspace
(when this exists) and used by package and release tools.

Example
'''''''

::

    <depends>roscpp</depends>


.. _stack_build_depends_tag:

<build_depends> (multiple)
--------------------------

Test
''''

Declares a ROS dep key that this stack depends on at build-time.
Used to determine dependency ordering of current workspace
(when this exists) and used by package and release tools.

Example
'''''''

::

    <build_depends>genmsg</build_depends>


.. _stack_build_type_tag:

<build_type>
------------

Text
''''

The build type determines the debian rules file to use.
Options: ``cmake``, ``autotools``, or ``python_distutils``.
See bloom/bin/em for the definitions.
Defaults to ``cmake``.

Attributes
''''''''''

 ``file="local_path_to_rules.em_file"`` *(optional)*
  If the value is ``custom`` then that file path has to be given. E.g. ``./bloom/rules.em``


Example
'''''''

::

    <build_type>autotools</build_type>
    <build_type file="./bloom/rules.em">custom</build_type>


.. _stack_message_generator_tag:

<message_generator>
-------------------

Defines the 'tag' for the language bindings generated by this package,
-e.g. in ``gencpp`` this is set to ``cpp``.

Example
'''''''

::

    <message_generator>cpp</message_generator>
