# Log levels
# 0 Normal use
# 1 Rosbuild developer use (Stuff being developed)
# 2 Rosbuild developer use (Stuff working)
# 3 Also Print True Assert Statements

function(LOG ARG_LEVEL ARG_MSG)
  if(NOT ROSBUILD_LOG)
    set(ROSBUILD_LOG 0)
  endif()

  if(NOT ARG_LEVEL GREATER ${ROSBUILD_LOG})
    message(STATUS "*${ARG_LEVEL}* ${ARG_MSG}")
  endif()
endfunction()