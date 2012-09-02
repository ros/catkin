.. _variables:

Variables
=========

Environment
-----------

.. cmake:data:: CATKIN_ENV

   The path to the shell script ``env.sh`` that will execute its
   arguments inside the catkin environment.  CMake that executes shell
   commands (e.g. as part of ``add_custom_command``) should use this
   rather than wrangling environment explicitly.

Install destinations
--------------------

All destination variable are meant to be used with the CMake ``install()`` macro as an ``DESTINATION`` argument.
They only contain relative paths and are supposed to be relative to the ``${CMAKE_INSTALL_PREFIX}``.

.. cmake:data:: PROJECT_INCLUDE_DESTINATION

   This is set to ``include``.

.. cmake:data:: PROJECT_LIB_DESTINATION

   This is set to ``lib``.

.. cmake:data:: PROJECT_LIBEXEC_DESTINATION

   This is set to ``lib/${PROJECT_NAME}``.

.. cmake:data:: PROJECT_PYTHON_DESTINATION

   This is set to ``lib/pythonX.Y/dist-packages/${PROJECT_NAME}``.

.. cmake:data:: PROJECT_SHARE_DESTINATION

   This is set to ``share/${PROJECT_NAME}``.

.. cmake:data:: GLOBAL_BIN_DESTINATION

   This is set to ``bin``.
   This destination should only be used for the core ROS binaries.
   If you are unsure if you should use this destination use ``PROJECT_LIBEXEC_DESTINATION`` instead.

Build space
-----------

.. cmake:data:: CATKIN_BUILD_PREFIX

   This is set to ``${CMAKE_BINARY_DIR}/buildspace`` by either ``catkin/CMakeLists.txt`` or ``catkin/cmake/catkinConfig.cmake`` and is the analogue to ``CMAKE_PREFIX_PATH``.
   Since the layout of the both folders ``CATKIN_BUILD_PREFIX`` and ``CMAKE_PREFIX_PATH`` is identical you can append any of the above install destinations to the build prefix.
