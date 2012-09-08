#
# Register environment hooks which are executed by the setup script.
#
# For each shell in ``SHELLS``, ``<file_prefix>.<shell>.in`` in the
# directory ``DIRECTORY`` is expand to ``etc/catkin/profile.d/``,
# where it will be read by generated ``setup.<shell>``.
#
# The template can distinguish between build- and installspace
# using the boolean variables ``BUILDSPACE`` and ``INSTALLSPACE``
# which are either ``true`` or ``false``.
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
# :param file_prefix: the filename prefix
# :type file_prefix: string
# :param SHELLS: the shell extensions (i.e.: sh bat)
# :type SHELLS: list of strings
# :param DIRECTORY: the directory (default: ${CMAKE_CURRENT_SOURCE_DIR})
# :type DIRECTORY: string
# :param SKIP_INSTALL: if specified the env hooks are only generated
#   in the buildspace but not installed
# :type SKIP_INSTALL: option
#
# @public
#
function(catkin_add_env_hooks file_prefix)
  parse_arguments(ARG "DIRECTORY;SHELLS" "SKIP_INSTALL" ${ARGN})

  # create directory if necessary
  if(NOT IS_DIRECTORY ${CATKIN_BUILD_PREFIX}/etc/catkin/profile.d)
    file(MAKE_DIRECTORY ${CATKIN_BUILD_PREFIX}/etc/catkin/profile.d)
  endif()

  if(NOT ARG_DIRECTORY)
    set(ARG_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  endif()

  foreach(shell ${ARG_SHELLS})
    set(ENV_HOOK ${file_prefix}.${shell})
    assert_file_exists(${ARG_DIRECTORY}/${ENV_HOOK}.in "User-supplied environment file '${ARG_DIRECTORY}/${ENV_HOOK}.in' missing")

    # generate environment hook for buildspace
    set(BUILDSPACE true)
    set(INSTALLSPACE false)
    configure_file(${ARG_DIRECTORY}/${ENV_HOOK}.in
      ${CATKIN_BUILD_PREFIX}/etc/catkin/profile.d/${ENV_HOOK})

    # generate and install environment hook for installspace
    set(BUILDSPACE false)
    set(INSTALLSPACE true)
    configure_file(${ARG_DIRECTORY}/${ENV_HOOK}.in
      ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${ENV_HOOK})
    if(NOT ${ARG_SKIP_INSTALL})
      install(FILES ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${ENV_HOOK}
        DESTINATION etc/catkin/profile.d)
    endif()
  endforeach()
endfunction()
