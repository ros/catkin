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
# TODO: remove the LDFLAGS override.  It's here to avoid esoteric problems
# of this sort:
#  https://code.ros.org/trac/ros/ticket/2977
#  https://code.ros.org/trac/ros/ticket/3842
export LDFLAGS=

%:
	dh  $@@

override_dh_auto_configure:
	# In case we're installing to a non-standard location, look for a setup.sh
	# in the install tree that was dropped by catkin, and source it.  It will
	# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
	if [ -f "@(INSTALL_PREFIX)/setup.sh" ]; then . "@(INSTALL_PREFIX)/setup.sh"; fi && \
	dh_auto_configure -- \
		-DCMAKE_INSTALL_PREFIX="@(INSTALL_PREFIX)" \
		-DCMAKE_PREFIX_PATH="@(INSTALL_PREFIX)"

override_dh_auto_build:
	# In case we're installing to a non-standard location, look for a setup.sh
	# in the install tree that was dropped by catkin, and source it.  It will
	# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
	if [ -f "@(INSTALL_PREFIX)/setup.sh" ]; then . "@(INSTALL_PREFIX)/setup.sh"; fi && \
	dh_auto_build

override_dh_auto_test:
	# In case we're installing to a non-standard location, look for a setup.sh
	# in the install tree that was dropped by catkin, and source it.  It will
	# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
	echo -- Running tests. Even if one of them fails the build is not canceled.
	if [ -f "@(INSTALL_PREFIX)/setup.sh" ]; then . "@(INSTALL_PREFIX)/setup.sh"; fi && \
	dh_auto_test || true

override_dh_shlibdeps:
	# In case we're installing to a non-standard location, look for a setup.sh
	# in the install tree that was dropped by catkin, and source it.  It will
	# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
	if [ -f "@(INSTALL_PREFIX)/setup.sh" ]; then . "@(INSTALL_PREFIX)/setup.sh"; fi && \
	dh_shlibdeps -l$(CURDIR)/debian/@(Package)/@(INSTALL_PREFIX)/lib/
