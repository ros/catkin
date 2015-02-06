#
# Download a file containing data from a URL.
#
# It is commonly used to download larger data files which should not be
# stored in the repository.
#
# .. note:: It is not recommended to rely on downloaded data during
#   a configure / make cycle since this prevents building the package
#   when no network connectivity is available.
#
# .. note:: The target will be registered as a dependency
#   of the "download_extra_data" target.
#
# :param target: the target name
# :type target: string
# :param url: the url to download
# :type url: string
#
# :param DESTINATION: the directory where the file is downloaded to
#   (default: ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION})
# :type DESTINATION: string
# :param FILENAME: the filename of the downloaded file
#   (default: the basename of the url)
# :type FILENAME: string
# :param MD5: the expected md5 hash to compare against
#   (default: empty, skipping the check)
# :type MD5: string
# :param EXCLUDE_FROM_ALL: exclude this download from the 'all' build target
#   (default: true, excluding it from the all target)
# :type EXCLUDE_FROM_ALL: bool
# :param REQUIRED: require the file to be available for the build to succeed
#   (default: true, requiring the file)
# :type REQUIRED: bool
#
# @public
function(catkin_download target url)
  cmake_parse_arguments(ARG ""
    "DESTINATION;FILENAME;MD5;EXCLUDE_FROM_ALL;REQUIRED" "" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "catkin_download() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT ARG_DESTINATION)
    set(ARG_DESTINATION ${PROJECT_BINARY_DIR})
  endif()

  if(NOT ARG_FILENAME)
    get_filename_component(ARG_FILENAME ${url} NAME)
  endif()

  set(required "")
  if(DEFINED ARG_REQUIRED)
    if(NOT ARG_REQUIRED)
      set(required "--non-essential")
    endif()
  endif()

  set(output "${ARG_DESTINATION}/${ARG_FILENAME}")

  set(DOWNLOAD_CHECKMD5_SCRIPT "${catkin_EXTRAS_DIR}/test/download_checkmd5.py")

  # With this, the command is always called, even when the output is up to date.
  # this is because we want to check the md5 sum if it's given, and redownload
  # the target if the md5 sum does not match.
  add_custom_target(${target}
    COMMAND ${DOWNLOAD_CHECKMD5_SCRIPT} ${url} ${output} ${ARG_MD5} ${required}
    VERBATIM)

  if(TARGET download_extra_data)
    add_dependencies(download_extra_data ${target})
  endif()

  if(DEFINED ARG_EXCLUDE_FROM_ALL)
    set_target_properties(${target} PROPERTIES
      EXCLUDE_FROM_ALL ${ARG_EXCLUDE_FROM_ALL})
  endif()

endfunction()

#
# Download a file containing data from a URL.
#
# It is commonly used to download larger data files which should not be
# stored in the repository.
#
# .. note:: The target will be registered as a dependency
#   of the "download_extra_data" target and the all target
#
# :param target: the target name
# :type target: string
# :param url: the url to download
# :type url: string
#
# :param DESTINATION: the directory where the file is downloaded to
#   (default: ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION})
# :type DESTINATION: string
# :param FILENAME: the filename of the downloaded file
#   (default: the basename of the url)
# :type FILENAME: string
# :param MD5: the expected md5 hash to compare against
#   (default: empty, skipping the check)
# :type MD5: string
# :param REQUIRED: require the file to be available for the build to succeed
#   (default: true, requiring the file)
# :type REQUIRED: bool
#
# @public
function(catkin_download_build_data)
  catkin_download(${ARGN} EXCLUDE_FROM_ALL false)
endfunction()

#
# Download a file containing data from a URL.
#
# It is commonly used to download larger data files which should not be
# stored in the repository, and which are not required as part of the build
#
# .. note:: The target will be registered as a dependency
#   of the "download_extra_data" target
#
# :param target: the target name
# :type target: string
# :param url: the url to download
# :type url: string
#
# :param DESTINATION: the directory where the file is downloaded to
#   (default: ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION})
# :type DESTINATION: string
# :param FILENAME: the filename of the downloaded file
#   (default: the basename of the url)
# :type FILENAME: string
# :param MD5: the expected md5 hash to compare against
#   (default: empty, skipping the check)
# :type MD5: string
# :param REQUIRED: require the file to be available for the build to succeed
#   (default: true, requiring the file)
# :type REQUIRED: bool
#
# @public
function(catkin_download_extra_data)
  catkin_download(${ARGN} EXCLUDE_FROM_ALL true)
endfunction()

if(NOT TARGET download_extra_data)
  add_custom_target(download_extra_data)
endif()
