#!/usr/bin/make -f
# -*- makefile -*-

# Uncomment this to turn on verbose mode.
@{
from sys import version_info as v
pyversion="%u.%u" % (v.major, v.minor)
}
export DH_VERBOSE=1
export DH_OPTIONS=-v
# this is the --install-layout=deb variety
export PYTHONPATH=@(CMAKE_INSTALL_PREFIX)/lib/python@(pyversion)/@(PYTHON_PACKAGES_DIR)

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
	dh_auto_install -Spython_distutils -- \
		--prefix="@(CMAKE_INSTALL_PREFIX)" --install-layout=deb

override_dh_auto_build:
	dh_auto_build -Scmake
	dh_auto_build -Spython_distutils


