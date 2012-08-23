#
#   :param globexpr: Glob expression (shell style)
#
#   For each file `F` in subdirectories of ``CMAKE_CURRENT_SOURCE_DIR``
#   that (recursively) match globbing expression `globexpr`, install
#   `F` to ``share/P/F``, where ``P`` is the name of the parent
#   directory of `F`
#
#   .. rubric:: Example
#
#   For a directory containing::
#
#     src/
#       CMakeLists.txt
#       foo/
#         bar.txt
#       shimmy/
#         baz/
#           bam.txt
#
#   A call to ``install_matching_to_share(b??.txt)`` in
#   ``src/CMakeLists.txt`` will create an installation of::
#
#     <CMAKE_INSTALL_PREFIX>/
#       share/
#         foo/
#           bar.txt
#         baz/
#           bam.txt
function(install_matching_to_share GLOBEXPR)
  file(GLOB_RECURSE globbed
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    ${GLOBEXPR})

  foreach(file ${globbed})
    get_filename_component(pathonly ${file} PATH)
    set(package ${CATKIN_CURRENT_STACK})
    if(pathonly)
      get_filename_component(package ${pathonly} NAME)
    endif()
    install(FILES ${file}
      DESTINATION share/${CATKIN_CURRENT_STACK})
  endforeach()
endfunction()
