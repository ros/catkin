#!/usr/bin/make -f
# -*- makefile -*-
# Sample debian/rules that uses debhelper.
# This file was originally written by Joey Hess and Craig Small.
# As a special exception, when this file is copied by dh-make into a
# dh-make output file, you may use that output file without restriction.
# This special exception was added by Craig Small in version 0.37 of dh-make.

# Uncomment this to turn on verbose mode.
export DH_VERBOSE=1
export DH_OPTIONS=-v

# include $(catkin_DIR)/debian.rules

%:
       dh  $@@

override_dh_auto_configure:
       dh_auto_configure -Scmake -- \
               -DCMAKE_INSTALL_PREFIX="@(CMAKE_INSTALL_PREFIX)" \
               -DCMAKE_PREFIX_PATH="@(CMAKE_PREFIX_PATH)" \
               -DCATKIN_PACKAGE_PREFIX="@(CATKIN_PACKAGE_PREFIX)" \
               -DCATKIN=YES

       dh_auto_configure -Spython_distutils

override_dh_auto_install:
       dh_auto_install -Scmake
       dh_auto_install -Spython_distutils -- --prefix="@(CMAKE_INSTALL_PREFIX)"

override_dh_auto_build:
       dh_auto_build -Scmake
       dh_auto_build -Spython_distutils


