#!/bin/bash -ex

TOP=$(cd `dirname $0` ; pwd)

[ $# -eq 2 ] || { /bin/echo "usage: $0 [srcdir] [scratchdir]" ; exit 1 ; }

SRC=$1
cd $SRC
SRC=$(/bin/pwd)
cd ..

SCRATCHDIR=$2
mkdir -p $SCRATCHDIR || /bin/true
cd $SCRATCHDIR
SCRATCHDIR=$(/bin/pwd) #abspath
cd ..

BUILD=$SCRATCHDIR/build
mkdir -p $BUILD

INSTALL=$SCRATCHDIR/install
mkdir -p $INSTALL

#EAR: CMAKE_PREFIX_PATH appears to be ignored if passed as a cmake variable...
export CMAKE_PREFIX_PATH=$INSTALL
CMAKE="cmake -DCMAKE_INSTALL_PREFIX=$INSTALL -DCATKIN=YES -DCATKIN_LOG=2"

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
    ros_comm ros_tutorials
do
    /bin/echo "======================= $proj"
    doone $proj
done

