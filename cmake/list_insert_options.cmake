#
# Append key[=value] elements from one list to another list if they are not already in the target list.
#
# :param listname: name of the destination list
# :type listname: string
#
# :param ARGN: options to be inserted
# :type ARGN: list key[=value] strings
#
# :param CONTEXT: context to be printen in case of erros
# :type CONTEXT: string
#
# :param LEVEL: level of the error message
# :type LEVEL: string
#
# Example:
# ::
#
#   list_insert_options(
#     listname
#     -DMY_DEF=1 -DMY_OTHER_DEF=2
#     CONTEXT example
#     LEVEL WARNING
#   )

function(list_insert_options listname)
  include (CMakeParseArguments)
  cmake_parse_arguments(ARG "" "CONTEXT;LEVEL" "" ${ARGN})
  if (NOT DEFINED ARG_LEVEL)
    set(ARG_LEVEL WARNING)
  endif()
  if (NOT DEFINED ARG_CONTEXT)
    set(ARG_CONTEXT "<unknown>")
  endif()
  foreach(item ${ARG_UNPARSED_ARGUMENTS})
      set(found "")
      # make a regex that matches key=[value] form
      string(REGEX REPLACE "^([^=]+)(=.*)$?" "^\\1(=.*)?$" expr ${item}) 

      foreach(elem ${${listname}})
        if(elem STREQUAL item)
          set(found TRUE)
          break()
        elseif(elem MATCHES ${expr})
          set(found TRUE)
          message(${ARG_LEVEL} "could not add '${item}' (from '${ARG_CONTEXT}'), already contained: '${elem}'")
          break()
        endif()
      endforeach()
      if(NOT found)
        list(APPEND ${listname} ${item})
      endif()
  endforeach()
  set(${listname} ${${listname}} PARENT_SCOPE) # copy over to parent scope
endfunction()
