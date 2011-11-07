function(assert VAR)
  if (NOT ${VAR})
    message(FATAL_ERROR "Assertion failed: ${VAR} (== ${${VAR}}")
  endif()
  log(3 "assert(${VAR}) okay (== ${${VAR}})")
endfunction()

