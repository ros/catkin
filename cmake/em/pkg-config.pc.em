prefix=@CMAKE_INSTALL_PREFIX
includedir=${prefix}/include
libdir=${prefix}/lib

Name: @(CATKIN_PACKAGE_PREFIX + PACKAGE_NAME)
Description: Description of @PACKAGE_NAME
Version: @PACKAGE_VERSION
Cflags: -I${includedir}
Libs: -L${libdir} @(' '.join(['-l%s' % lib for lib in PACKAGE_LIBRARIES.strip().split(';') if len(lib) > 0]))
Requires: @(PACKAGE_DEPENDS.replace(';', ' '))

