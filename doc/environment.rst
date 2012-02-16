.. _envfiles:

Environment files
=================

Catkin creates and installs files for env-setting convenience.  In the
buildspace there are ``setup.zsh`` and ``setup.bash``, these contain
variable zsh and bash specific tweaks.  They both import ``setup.sh``,
which contains bourne-shell variable settings.

.. _env.sh:
.. _setup.sh:
.. _setup.bash:
.. _setup.zsh:

.. index:: env.sh ; environment
.. index:: setup.sh ; environment
.. index:: setup.bash ; environment
.. index:: setup.zsh ; environment

``env.sh``, pointed to by the cmake variable :cmake:data:`CATKIN_ENV`
is special; it executes its arguments in the environment created by
``setup.sh`` and returns.  Any custom commands executed by cmake
should do so via this script.  If executed without arguments, it will
spawn a subshell (using the current login shell, or ``$SHELL``) in the
development environment.

.. warning:: That ``env.sh`` executes a subshell means that your shell
             initialization files will be read `after` the environment
             is set by catkin's env hooks; this means that what these
             files want to do may get clobbered.  If you want this
             entering- and exiting of subshells mechanism to work
             correctly, be careful what you do in your ``.bashrc`` or
             ``.zshrc``.  Understand what your shell does when it is
             invoked both interactively and non-interactively.  The
             simpler mechanism is simply to 'source' the appropriate
             ``setup`` file, provided you are satisfied with the
             irreversible changes that these files make to your
             current environment.  All depends on your workflow.

.. rubric:: Environment hooks

Projects can, via the :cmake:macro:`catkin_add_env_hooks` macro, add
sh code to be executed by ``setup.sh`` (and by extension
``setup.bash`` and friends).  If you need to add things to the
environment, this is probably the place to do it.  Don't get fancy:
the contents of these scripts must be interpretable by all members of
the bourne shell family.  Be safe and ensure that ``/bin/dash`` is
okay with them.

**NOTE**: These environment hooks are only for variable settings.
Shell aliases and functions


