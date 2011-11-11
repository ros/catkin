function(assert VAR)
  if (NOT ${VAR})
    message(FATAL_ERROR "Assertion failed: ${VAR} (value is ${${VAR}})")
  endif()
  log(3 "assert(${VAR}) okay (== ${${VAR}})")
endfunction()

function(assert_file_exists FILENAME MESSAGE)
  if (NOT FILENAME)
    message(FATAL_ERROR
      "Assertion failed:  check for file existence, but filename (${FILENAME}) unset. Message: ${MESSAGE}")
  endif()
  if (NOT EXISTS ${FILENAME})
    message(FATAL_ERROR "Assertion failed:  file '${FILENAME}' does not exist.  Message: ${MESSAGE}")
  endif()
endfunction()

