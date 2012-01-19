#!/usr/bin/make -f
# -*- makefile -*-
#
# rules file for rosbuild-compatibility
#
export DH_VERBOSE=1
export DH_OPTIONS=-v --buildsystem=cmake
export CMAKE_PREFIX_PATH=@(INSTALL_PREFIX)


%:
	dh  $@@

override_dh_auto_configure:
	dh_auto_configure -- \
		-DCATKIN=YES \
		-DCMAKE_INSTALL_PREFIX="@(INSTALL_PREFIX)" \
		-DCMAKE_PREFIX_PATH="@(INSTALL_PREFIX)"

override_dh_auto_test:
	echo -- Running tests. Even if one of them fails the build is not canceled.
	dh_auto_test || true
