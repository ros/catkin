Name: @(CATKIN_PACKAGE_PREFIX + PROJECT_NAME)
Description: Description of @PROJECT_NAME
Version: @PROJECT_VERSION
Cflags: @(' '.join(['-I%s' % include for include in PROJECT_ABSOLUTE_INCLUDE_DIRS.split(';')]))
Libs: -L@PROJECT_SPACE_DIR/lib @(' '.join(['-l%s' % lib for lib in PROJECT_LIBRARIES.split(';') if len(lib) > 0]))
Requires: @(PROJECT_DEPENDS.replace(';', ' '))
