#
# Download a file from a URL.
#
# .. note:: It is not recommended to rely on downloaded data during
#   a configure / make cycle since this prevents building the package
#   when no network connectivity is available.
#
# :param target: the target name
# :type target: string
# :param url: the url to download
# :type url: string

# :param DESTINATION: the directory where the file is downloaded to
#   (default: ${PROJECT_BINARY_DIR})
# :type DESTINATION: string
# :param FILENAME: the filename of the downloaded file
#   (default: the basename of the url)
# :type FILENAME: string
# :param MD5: the expected md5 hash to compare against
#   (default: empty, skipping the check)
# :type MD5: string
#
# @public
function(catkin_download target url)
  cmake_parse_arguments(ARG "" "DESTINATION;FILENAME;MD5" "" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "catkin_download() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()
  if(NOT ARG_DESTINATION)
    set(ARG_DESTINATION ${PROJECT_BINARY_DIR})
  endif()
  if(NOT ARG_FILENAME)
    get_filename_component(ARG_FILENAME ${url} NAME)
  endif()
  set(output "${ARG_DESTINATION}/${ARG_FILENAME}")
  add_custom_command(OUTPUT ${output}
    COMMAND ${PYTHON_EXECUTABLE} ${catkin_EXTRAS_DIR}/test/download_checkmd5.py ${url} ${output} ${ARG_MD5}
    VERBATIM)
  add_custom_target(${target} DEPENDS ${output})
endfunction()
