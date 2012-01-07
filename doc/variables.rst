Variables
=========

.. cmake:data:: PROJECT_SHARE_INSTALL_PREFIX

   This is set to ``CMAKE_INSTALL_PREFIX/share/${PROJECT_NAME}`` by
   default.  For use with cmake ``install()`` macro.

.. cmake:data:: CATKIN_ENV

   The path to the shell script ``env.sh`` that will execute its
   arguments inside the catkin environment.  CMake that executes shell
   commands (e.g. as part of ``add_custom_command`` should use this
   rather than wrangling environment explicitly.
