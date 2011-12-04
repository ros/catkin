#!/bin/bash -ex

TOP=$(cd `dirname $0` ; pwd)
BUILD=$TOP/pkgbuild

CMAKE="cmake -DCMAKE_INSTALL_PREFIX=/opt/ros/fuerte -DCMAKE_PREFIX_PATH=/opt/ros/fuerte -DCATKIN=YES -DCATKIN_DEB_SNAPSHOTS=YES -DCATKIN_PACKAGE_PREFIX=ros-fuerte-"
DESTDIR=$TOP/DESTDIR
CATKIN_DEB_SNAPSHOT_VERSION=$(date +%Y%m%d-%H%M%z)
export CATKIN_DEB_SNAPSHOT_VERSION

sudo dpkg -r ros-fuerte\* || /bin/true
sudo rm -rf /opt/ros/fuerte/* || /bin/true
sudo mkdir -p /opt/ros/fuerte
sudo chown -R `whoami` /opt/ros/fuerte

rm -rf $BUILD
mkdir -p $BUILD
cd $BUILD

fatbuild ()
{
    mkdir $BUILD/buildall
    pushd $BUILD/buildall
    $CMAKE '-DCATKIN_DPKG_BUILDPACKAGE_FLAGS=-d;-S;-kBE0A7693' ../../src/test.rosinstall
    for distro in lucid maverick natty oneiric
    do
        make VERBOSE=1 CATKIN_DEB_SNAPSHOT_VERSION=$CATKIN_DEB_SNAPSHOT_VERSION CATKIN_DEBIAN_DISTRIBUTION=$distro catkin_test-gendebian
    done
    popd
}

fatbuild
