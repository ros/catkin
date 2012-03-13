
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

