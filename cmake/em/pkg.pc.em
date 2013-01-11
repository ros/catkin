Name: @(CATKIN_PACKAGE_PREFIX + PROJECT_NAME)
Description: Description of @PROJECT_NAME
Version: @PROJECT_VERSION
Cflags: @(' '.join(['-I%s' % include for include in PROJECT_ABSOLUTE_INCLUDE_DIRS.split(';')]))
Libs: -L@PROJECT_SPACE_DIR/lib @(' '.join([lib for lib in PKG_CONFIG_LIBRARIES_WITH_PREFIX.split(';')]))
Requires: @(PROJECT_CATKIN_DEPENDS.replace(';', ' '))
