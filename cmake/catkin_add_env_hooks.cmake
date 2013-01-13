#
# Register environment hooks which are executed by the setup script.
#
# For each shell in ``SHELLS``, macro searches for first of
# ``<file_prefix>.<shell>, ``<file_prefix>.<shell>.em`` or
# ``<file_prefix>.<shell>.in`` in the
# directory ``DIRECTORY``. Plain shells, will be copied to, templates
# are expanded to ``etc/catkin/profile.d/``,
# where it will be read by global generated ``setup.<shell>``.
#
# The template can distinguish between build- and installspace
# using the boolean variables ``DEVELSPACE`` and ``INSTALLSPACE``
# which are either ``true`` or ``false``.
# E.g. @[if DEVELSPACE]@ ... @[end if]@ for .em
#
# .. note:: Note the extra ".in" that must appear in the filename
#   that does not appear in the argument.
#
# .. note:: These files will share a single directory with other
#   packages that choose to install env hooks.  Be careful to give
#   the file a unique name.  Typically ``NN.name.<shell>`` is used,
#   where NN can define when something should be run (the files are
#   read in alphanumeric order) and the name serves to disambiguate
#   in the event of collisions.
#
# Example::
#
#   catkin_add_env_hooks(my_prefix SHELLS bash tcsh zsh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
#
# looks for files env-hooks/my_prefix.[bash|tcsh|zsh].[in|em]
#
# :param file_prefix: the filename prefix
# :type file_prefix: string
# :param SHELLS: the shell extensions (e.g.: sh bat bash zsh tcsh)
# :type SHELLS: list of strings
# :param DIRECTORY: the directory (default: ${CMAKE_CURRENT_SOURCE_DIR})
# :type DIRECTORY: string
# :param SKIP_INSTALL: if specified the env hooks are only generated
#   in the devel space but not installed
# :type SKIP_INSTALL: option
#
# @public
#
function(catkin_add_env_hooks file_prefix)
  cmake_parse_arguments(ARG "SKIP_INSTALL" "DIRECTORY" "SHELLS" ${ARGN})

  # create directory if necessary
  if(NOT IS_DIRECTORY ${CATKIN_DEVEL_PREFIX}/etc/catkin/profile.d)
    file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/etc/catkin/profile.d)
  endif()

  if(NOT ARG_DIRECTORY)
    set(ARG_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  endif()

  foreach(shell ${ARG_SHELLS})
    set(ENV_HOOK ${file_prefix}.${shell})
    set(base ${ARG_DIRECTORY}/${ENV_HOOK})

    # generate environment hook for devel space
    if (EXISTS ${base})
      # copy plain file
      file(COPY ${base} DESTINATION ${CATKIN_DEVEL_PREFIX}/etc/catkin/profile.d)
    elseif(EXISTS ${base}.em OR EXISTS ${base}.develspace.em)
      # evaluate em template
      set(DEVELSPACE True)
      set(INSTALLSPACE False)
      if(EXISTS ${base}.em)
        set(em_template ${base}.em)
      else()
        set(em_template ${base}.develspace.em)
      endif()
      em_expand(${catkin_EXTRAS_DIR}/templates/env-hook.context.py.in
        ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/${ENV_HOOK}.develspace.context.py
        ${em_template}
        ${CATKIN_DEVEL_PREFIX}/etc/catkin/profile.d/${ENV_HOOK})
    elseif(EXISTS ${base}.in)
      # evaluate in template
      set(DEVELSPACE true)
      set(INSTALLSPACE false)
      configure_file(${base}.in
        ${CATKIN_DEVEL_PREFIX}/etc/catkin/profile.d/${ENV_HOOK}
        @ONLY)
    else()
      message(FATAL_ERROR "catkin_add_env_hooks() could not find environment hook.  Either '${base}', '${base}.em', '${base}.develspace.em' or '${base}.in' must exist.")
    endif()

    # generate and install environment hook for installspace
    if (EXISTS ${base})
      # install plain file
      if(NOT ${ARG_SKIP_INSTALL})
        install(FILES ${base}
          DESTINATION etc/catkin/profile.d)
      endif()
    elseif(EXISTS ${base}.em OR EXISTS ${base}.installspace.em)
      # evaluate em template and install
      set(DEVELSPACE False)
      set(INSTALLSPACE True)
      if(EXISTS ${base}.em)
        set(em_template ${base}.em)
      else()
        set(em_template ${base}.installspace.em)
      endif()
      em_expand(${catkin_EXTRAS_DIR}/templates/env-hook.context.py.in
        ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/${ENV_HOOK}.installspace.context.py
        ${em_template}
        ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${ENV_HOOK})
      if(NOT ${ARG_SKIP_INSTALL})
        install(FILES ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${ENV_HOOK}
          DESTINATION etc/catkin/profile.d)
      endif()
    elseif(EXISTS ${base}.in)
      # evaluate in template and install
      set(DEVELSPACE false)
      set(INSTALLSPACE true)
      configure_file(${base}.in
        ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${ENV_HOOK}
        @ONLY)
      if(NOT ${ARG_SKIP_INSTALL})
        install(FILES ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${ENV_HOOK}
          DESTINATION etc/catkin/profile.d)
      endif()
    endif()
  endforeach()

  # refresh environment cache
  safe_execute_process(COMMAND ${GENERATE_ENVIRONMENT_CACHE_COMMAND})
endfunction()
