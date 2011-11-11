# Log levels
# 0 Normal use
# 1 Catkin developer use (Stuff being developed)
# 2 Catkin developer use (Stuff working)
# 3 Also Print True Assert Statements

function(LOG ARG_LEVEL ARG_MSG)
  if(NOT CATKIN_LOG)
    set(CATKIN_LOG 0)
  endif()

  if(NOT ARG_LEVEL GREATER ${CATKIN_LOG})
    message(STATUS "*${ARG_LEVEL}* ${ARG_MSG}")
  endif()
endfunction()


macro(info MSG)
  message(STATUS "~~ ${PROJECT_NAME} ${MSG}")
endmacro()