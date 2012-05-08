
# BUILD_SHARED_LIBS is a global cmake variable (usually defaults to on) 
# that determines the build type of libraries:
#   http://www.cmake.org/cmake/help/cmake-2-8-docs.html#variable:BUILD_SHARED_LIBS
# It defaults to shared.
#
# Our only current major use case for static libraries is
# via the mingw cross compiler, though embedded builds
# could be feasibly built this way also (largely untested).

# Cached variable
option(BUILD_SHARED_LIBS "Build dynamically-linked binaries" ON)

function(configure_shared_library_build_settings)
  if (BUILD_SHARED_LIBS)
    message(STATUS "BUILD_SHARED_LIBS is on.")
    add_definitions(-DROS_BUILD_SHARED_LIBS=1)
  else()
    message(STATUS "BUILD_SHARED_LIBS is off.")
  endif()
endfunction()

# Windows/cmake make things difficult if building dll's. 
# These use RUNTIME_OUTPUT_DIRECTORY (aka bin) and can't be 
# distinguished from exe's. We want exe output directory to be
# in the package's local folder (see catkin_project) - but dlls 
# to be public - ${CMAKE_BINARY_DIR}/bin.
# 
# This is an awful hack - we want to move away from redefining
# add_library style calls, but necessary until a better solution
# is available for windows. Alternatives are to set_target_properties
# on every lib (painful) or to make exe's public (name conflicts
# bound to arise).
if (BUILD_SHARED_LIBS)
  if (MSVC)
    function(add_library library)
      # Check if its an external, imported library (e.g. boost libs via cmake module definition)
      list(FIND ARGN "IMPORTED" FIND_POS)
      _add_library(${ARGV0} ${ARGN})
      if( ${FIND_POS} EQUAL -1 )
        # It is not imported, add our custom copy rule
        add_custom_command(TARGET ${ARGV0} POST_BUILD
                 #cmake -E copy_if_different ${ARGV0}.dll ${CMAKE_BINARY_DIR}/bin # Doesn't handle regexp, i.e. dll*
                 COMMAND if exist "${PROJECT_BINARY_DIR}/bin/${ARGV0}.dll" ( cp bin/*dll* ${CMAKE_BINARY_DIR}/bin )
                 WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
                 )
        # Quite likely, linux guys hacking packages will not think to set the runtime destination to bin
        # for dll's and this will create a huge headache in porting. Just do it here for now.
        # From cmake docs - 'Installing a target with EXCLUDE_FROM_ALL set to true has undefined behavior.'
        get_target_property(${ARGV0}_EXCLUDE_FROM_INSTALL ${ARGV0} EXCLUDE_FROM_ALL)
        if ( NOT ${${ARGV0}_EXCLUDE_FROM_INSTALL} )  
            install(TARGETS ${ARGV0}
                    RUNTIME DESTINATION bin
                    ARCHIVE DESTINATION lib
                    LIBRARY DESTINATION lib 
            )
        endif()
      endif()
    endfunction()
  endif(MSVC)
endif(BUILD_SHARED_LIBS)
