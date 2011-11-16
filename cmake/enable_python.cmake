function(enable_python pkg_name)
  configure_file(${catkin_EXTRAS_DIR}/__init__.py.in
                 ${CMAKE_BINARY_DIR}/gen/py/${pkg_name}/__init__.py
                 @ONLY)
endfunction()