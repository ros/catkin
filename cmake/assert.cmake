function(assert VAR)
  if (NOT ${VAR})
    message(FATAL_ERROR "Assertion failed: ${VAR} (== ${${VAR}}")
  endif()
  log(3 "assert(${VAR}) okay (== ${${VAR}})")
endfunction()

function(assert_file_exists FILENAME MESSAGE)
  if (NOT EXISTS ${FILENAME})
    message(FATAL_ERROR "Assertion failed:  file '${FILENAME}' does not exist.  Message: ${MESSAGE}")
  endif()
endfunction()

