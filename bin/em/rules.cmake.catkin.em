#!/usr/bin/make -f
# -*- makefile -*-
#
# rules file for rosbuild-compatibility
#
export DH_VERBOSE=1
export DH_OPTIONS=-v

%:
	dh  $@@

override_dh_auto_configure:
	dh_auto_configure -Scmake -- \
		-DCATKIN=YES \
		-DCMAKE_INSTALL_PREFIX="@(INSTALL_PREFIX)" \
		-DCMAKE_PREFIX_PATH="@(INSTALL_PREFIX)"

