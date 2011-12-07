#!/bin/bash -ex

TOP=$(cd `dirname $0` ; pwd)
BUILD=$TOP/pkgbuild

CMAKE="cmake -DCMAKE_INSTALL_PREFIX=/opt/ros/fuerte -DCMAKE_PREFIX_PATH=/opt/ros/fuerte -DCATKIN=YES -DCATKIN_LOG=2 -DCATKIN_DEB_SNAPSHOTS=YES -DCATKIN_PACKAGE_PREFIX=ros-fuerte-"
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

SRC=$TOP/src

rm $SRC/*.tar.gz $SRC/*.deb $SRC/*.changes $SRC/*.dsc || /bin/true

doone () {
    pkg=$1
    mkdir $BUILD/$pkg
    pushd $BUILD/$pkg
    SRC=../../src/$pkg
    $CMAKE $SRC
    make VERBOSE=1
    make VERBOSE=1 install
    for distro in lucid maverick natty oneiric
    do
        make CATKIN_DEBIAN_DISTRIBUTION=$distro $pkg-gendebian
    done
    popd
}

fatbuild ()
{
    mkdir $BUILD/buildall
    pushd $BUILD/buildall
    cmake ../../src -DCMAKE_INSTALL_PREFIX=/opt/ros/fuerte
    make
    make install
    popd
}

fatbuild

doone catkin
doone genmsg
doone gencpp
doone genpy
doone std_msgs
doone common_msgs
doone roscpp_core
doone ros_comm
doone nolangs
doone catkin_test

for i in $TOP/src/ros-fuerte-*.changes
do
    dput ppa:straszheim/ros $i || /bin/true
done
