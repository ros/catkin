function(set_once VAR VALUE)

  if (VAR)
    if (NOT ${${VAR}} STREQUAL ${VALUE})
      message(FATAL_ERROR "Attempt to change value of ${VAR} from ${${VAR}} to ${VALUE}")
    endif()
  endif()
  set(${VAR} ${VALUE} CACHE STRING "uhm")

endfunction()