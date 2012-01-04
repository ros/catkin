#!/bin/bash -ex

TOP=$(cd `dirname $0` ; pwd)
BUILD=$TOP/pkgbuild
PREFIX=$1

CMAKE="cmake -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_PREFIX_PATH=$PREFIX -DCATKIN=YES -DCATKIN_LOG=2"
DESTDIR=$TOP/DESTDIR

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
doone genpybindings
doone gentypelibxml

doone std_msgs
doone common_msgs
doone rospack
doone ros

doone roscpp_core
doone nolangs
doone catkin_test
doone ros_comm
doone ros_tutorials

