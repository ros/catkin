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
CMAKE="cmake -DCMAKE_PREFIX_PATH=$INSTALL -DCMAKE_INSTALL_PREFIX=$INSTALL"

doone () {
    pkg=$1
    mkdir -p $BUILD/$pkg
    cd $BUILD/$pkg
    $CMAKE $SRC/$proj
    make -j8
    make install
    cd ..
}

for proj in catkin rospkg rospack ros genmsg gencpp genpy genpybindings gentypelibxml roscpp_core std_msgs common_msgs ros_comm ros_tutorials
do
    /bin/echo "\n\n\n======================= $proj ======================="
    doone $proj
done

