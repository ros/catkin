#!/usr/bin/make -f

# Uncomment this to turn on verbose mode.
export DH_VERBOSE=1
export DH_OPTIONS=-v


%:
	dh  $@@

override_dh_auto_configure:
       dh_auto_configure -Scmake -- \
               -DCMAKE_INSTALL_PREFIX="@(CMAKE_INSTALL_PREFIX)" \
               -DCMAKE_PREFIX_PATH="@(CMAKE_INSTALL_PREFIX)" \
               -DCATKIN_PACKAGE_PREFIX="@(CATKIN_PACKAGE_PREFIX)" \
               -DCATKIN=YES
       dh_auto_configure -Spython_distutils

override_dh_auto_install:
       dh_auto_install -Scmake
       dh_auto_install -Spython_distutils -- --prefix="@(CMAKE_INSTALL_PREFIX)"

override_dh_auto_build:
       dh_auto_build -Scmake
       dh_auto_build -Spython_distutils


