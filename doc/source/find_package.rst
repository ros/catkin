
find_package() infrastructure
-----------------------------

CMake's ``find_package`` is the preferred method for packages to
communciate to CMake (and thereby to catkin) the libraries, include
directories and such that packages should use.

There are a couple of modes of operation of find_package (see the
CMake documentation), "module" mode and "config" mode.  "module" mode
is the one that uses cmake scripts named ``Find****.cmake``.  "config"
mode is the preferred mode, and it works differently.  

One reason we find the 'config mode' superior is that is supports
multiple simultaneous installed versions of packages.

For a package T, 'config mode' consists of two cmake files, both of
which are installed somewhere on cmake's ``CMAKE_PREFIX_PATH``.  The
first is 't-config-version.cmake'.  We find the most succinct form to
be like this::

  set(PACKAGE_VERSION_EXACT False)
  set(PACKAGE_VERSION_COMPATIBLE False)
  if("${PACKAGE_FIND_VERSION}" STREQUAL "")
    set(PACKAGE_VERSION_COMPATIBLE TRUE)
    return()
  endif()
  
  if("${PACKAGE_FIND_VERSION}" VERSION_EQUAL "9.9.9")
    set(PACKAGE_VERSION_EXACT True)
    set(PACKAGE_VERSION_COMPATIBLE True)
  endif()
  
  if("${PACKAGE_FIND_VERSION}" VERSION_LESS "9.9.9")
    set(PACKAGE_VERSION_COMPATIBLE True)
  endif()
  
where `9.9.9` is replaced by the numeric version of the package.  The
second file tells the caller what the assorted includes/libs are for
the package::





