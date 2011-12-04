TOP=$(cd `dirname $0` ; pwd)
BUILD=$TOP/build
SRC=$TOP/src

cleanup () {
    rm -rf $BUILD || /bin/true
    mkdir -p $BUILD

    sudo dpkg -r ros-fuerte\* || /bin/true
    sudo rm -rf /opt/ros/fuerte/* || /bin/true
    sudo mkdir -p /opt/ros/fuerte
    sudo chown -R `whoami` /opt/ros/fuerte
    
    rm $SRC/*.dsc || /bin/true
    rm $SRC/*.tar.gz || /bin/true
    rm $SRC/*.deb || /bin/true
    rm $SRC/*.changes || /bin/true
}