#
# parse_arguments() taken from
# http://www.itk.org/Wiki/CMakeMacroParseArguments
#
# @deprecated use CMakeParseArguments instead
#
macro(parse_arguments prefix arg_names option_names)
  message(WARNING "parse_arguments(prefix args options ${ARGN}) is deprecated. Use the corresponding CMake function instead:")
  message("  call 'include(CMakeParseArguments)' to include the CMake function")
  message("  Update the signature:")
  message("  instead of 'parse_arguments(prefix option_names arg_names ${ARGN})'")
  message("  it is 'cmake_parse_arguments(prefix option_names single_arg_names multi_arg_names ${ARGN}))")
  message("  and the variable containing not matched arguments must be changed form 'prefix_DEFAULT_ARGS' to 'prefix_UNPARSED_ARGUMENTS'")

  set(DEFAULT_ARGS)
  foreach(arg_name ${arg_names})
    set(${prefix}_${arg_name})
  endforeach()
  foreach(option ${option_names})
    set(${prefix}_${option} FALSE)
  endforeach()

  set(current_arg_name DEFAULT_ARGS)
  set(current_arg_list)
  foreach(arg ${ARGN})
    set(larg_names ${arg_names})
    list(FIND larg_names "${arg}" is_arg_name)
    if(is_arg_name GREATER -1)
      set(${prefix}_${current_arg_name} ${current_arg_list})
      set(current_arg_name ${arg})
      set(current_arg_list)
    else()
      set(loption_names ${option_names})
      list(FIND loption_names "${arg}" is_option)
      if(is_option GREATER -1)
        set(${prefix}_${arg} TRUE)
      else()
        set(current_arg_list ${current_arg_list} ${arg})
      endif()
    endif()
  endforeach()
  set(${prefix}_${current_arg_name} ${current_arg_list})
endmacro()
