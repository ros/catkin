#
# For each shell in ``SHELLS``, ``<fileprefix>.<shell>.in`` in the
# directory ``DIRECTORY`` is expand to ``etc/catkin/profile.d/``,
# where it will be read by generated ``setup.<shell>``.
#
# The template can distinguish between build- and installspace using
# the boolean parameters ``ENV_BUILDSPACE`` and ``ENV_INSTALLSPACE``
# which are either ``true`` or ``false``.
#
# .. note:: Note the extra ".in" that must appear in the filename
#    that does not appear in the argument.
#
# **NOTE** These files will share a single directory with other
# packages that choose to install env hooks.  Be careful to give the
# file a unique name.  Typically ``NNprojectname.sh`` is used, where
# NN can define when something should be run (the files are read in
# alphanumeric order) and ``projectname`` serves to disambiguate in
# the event of collision.
#
# :param SHELLS: list of shells (i.e.: sh bat)
# :param DIRECTORY: Directory of the env hooks (default ${CMAKE_CURRENT_SOURCE_DIR})
# :param SKIP_INSTALL: if specified the env hooks are only generated
#   in the buildspace but not installed
#
function(catkin_add_env_hooks ARG_ENV_HOOK)
  parse_arguments(ARG "DIRECTORY;SHELLS" "SKIP_INSTALL" ${ARGN})

  # create directory if necessary
  if(NOT IS_DIRECTORY ${catkin_BUILD_PREFIX}/etc/catkin/profile.d)
    file(MAKE_DIRECTORY ${catkin_BUILD_PREFIX}/etc/catkin/profile.d)
  endif()

  if(NOT ARG_DIRECTORY)
    set(ARG_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  endif()

  foreach(shell ${ARG_SHELLS})
    set(ENV_HOOK ${ARG_ENV_HOOK}.${shell})
    assert_file_exists(${ARG_DIRECTORY}/${ENV_HOOK}.in "User-supplied environment file '${ARG_DIRECTORY}/${ENV_HOOK}.in' missing")

    # generate environment hook for buildspace
    set(ENV_BUILDSPACE true)
    set(ENV_INSTALLSPACE false)
    configure_file(${ARG_DIRECTORY}/${ENV_HOOK}.in
      ${catkin_BUILD_PREFIX}/etc/catkin/profile.d/${ENV_HOOK})

    # generate and install environment hook for installspace
    set(ENV_BUILDSPACE false)
    set(ENV_INSTALLSPACE true)
    configure_file(${ARG_DIRECTORY}/${ENV_HOOK}.in
      ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${ENV_HOOK})
    if(NOT ${ARG_SKIP_INSTALL})
      install(FILES ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${ENV_HOOK}
        DESTINATION etc/catkin/profile.d)
    endif()
  endforeach()
endfunction()
