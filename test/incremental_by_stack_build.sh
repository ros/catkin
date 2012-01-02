#!/bin/bash -ex

TOP=$(cd `dirname $0` ; pwd)

SRC=$TOP/src

[ $# -eq 1 ] || { /bin/echo "usage: $0 [prefix]" ; exit 1 ; }

PREFIX=$1
mkdir -p $PREFIX
cd $PREFIX
PREFIX=$(/bin/pwd) #abspath

BUILD=$PREFIX/build
mkdir -p $BUILD

INSTALL=$PREFIX/install
mkdir -p $INSTALL

CMAKE="cmake -DCMAKE_INSTALL_PREFIX=$INSTALL -DCMAKE_PREFIX_PATH=$INSTALL -DCATKIN=YES -DCATKIN_LOG=2"

SRC=$TOP/src

doone () {
    pkg=$1
    mkdir $BUILD/$pkg
    pushd $BUILD/$pkg
    $CMAKE $SRC/$proj
    make VERBOSE=1
    make VERBOSE=1 install
    popd
}

for proj in catkin genmsg gencpp genpy genpybindings gentypelibxml \
    roscpp_core std_msgs common_msgs rospack ros \
    nolangs catkin_test ros_comm ros_tutorials
do
    /bin/echo "======================= $proj"
    doone $proj
done

