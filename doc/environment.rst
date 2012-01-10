Environment files
=================

Catkin creates and installs files for env-setting convenience.  In the
buildspace there are ``setup.zsh`` and ``setup.bash``, these contain
variable zsh and bash specific tweaks.  They both import ``setup.sh``,
which contains bourne-shell variable settings.

.. index:: env.sh ; environment

``env.sh``, pointed to by the cmake variable :cmake:data:`CATKIN_ENV` is
special; it executes its arguments in the environment created by
``setup.sh`` and returns.  Any custom commands executed by cmake
should do so via this script.

Projects can, via the :cmake:macro:`catkin_add_env_hooks` macro, add
sh code to be executed by ``setup.sh`` (and by extension
``setup.bash`` and friends).  If you need to add things to the
environment, this is probably the place to do it.  Don't get fancy:
the contents of these scripts must be interpretable by all members of
the bourne shell family.





