#!/usr/bin/make -f
# -*- makefile -*-
# Sample debian/rules that uses debhelper.
# This file was originally written by Joey Hess and Craig Small.
# As a special exception, when this file is copied by dh-make into a
# dh-make output file, you may use that output file without restriction.
# This special exception was added by Craig Small in version 0.37 of dh-make.

# Uncomment this to turn on verbose mode.
export DH_VERBOSE=1
export DH_OPTIONS=-v --buildsystem=cmake
export CMAKE_PREFIX_PATH=@(INSTALL_PREFIX)

%:
	dh  $@@

#
#  The CATKIN=YES is needed to make it possible for
#  ROS packages to build under rosmake and catkin
#
override_dh_auto_configure:
	dh_auto_configure -- \
		-DCMAKE_INSTALL_PREFIX="@(INSTALL_PREFIX)" \
		-DCMAKE_PREFIX_PATH="@(INSTALL_PREFIX)"

override_dh_auto_test:
	echo -- Running tests. Even if one of them fails the build is not canceled.
	dh_auto_test || true
