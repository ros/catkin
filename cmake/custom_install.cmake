function(_use_custom_install)
  # only redefine install function once
  if(NOT _CATKIN_USE_CUSTOM_INSTALL)
    set(_CATKIN_USE_CUSTOM_INSTALL TRUE PARENT_SCOPE)

    function(install)
      if(_CATKIN_SKIP_INSTALL_RULES)
        return()
      endif()
      _install(${ARGN})
    endfunction()
  endif()
endfunction()
