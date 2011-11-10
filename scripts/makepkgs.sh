#!/bin/bash -ex

TOP=$(cd `dirname $0`; pwd)
SRC=/home/straszheim/src

PREFIX=/tmp/PREFIX

SERIES=ros-genocidal-
DISTROS="lucid maverick natty oneiric"

clean () {
    pushd $SRC
    rm -f *.deb *.dsc *.build *.changes *.upload *.tar.gz
    popd
}

srcpackage () {
    dpkg-source -b $SRC/$1

}

debuildit () {
    cd $SRC/$1
    DISTRO=$2

    rm -rf .build
    mkdir .build

    if [ -f ./debian/changelog.em ] ; then
        $TOP/changelog_increment.py ./debian/changelog.em $DISTRO > .build/debian/changelog.
        cp ./debian/changelog.em .build/debian/changelog.aside
        cat ./debian/changelog.new ./debian/changelog.aside > ./debian/changelog.em
    fi

    cd .build
    cmake .. -DCMAKE_PREFIX_PATH=$PREFIX -DCMAKE_INSTALL_PREFIX=$PREFIX \
        -DCATKIN_PACKAGE_PREFIX=$SERIES \
        -DCATKIN_LINUX_DISTRIBUTIONS="$DISTRO" \
        -DCATKIN=YES

    cd $SRC/$1
    debuild \
        -e DH_VERBOSE=1 \
        -e DH_OPTIONS="-v -P$TOP/$1/pkg_tmpdir" \
        -e DEB_SOURCE_DIR=`pwd` \
        -e DEB_BUILD_DIR=$DEB_BUILD_DIR_PFX/$1 \
        -e DEB_DEBS_DIR=$TOP/debs \
        -e CMAKE_INSTALL_PREFIX=$PREFIX \
        -e CMAKE_PREFIX_PATH=$PREFIX \
        -e CATKIN_PACKAGE_PREFIX=$SERIES \
        -e CATKIN_LINUX_DISTRIBUTIONS="$DISTRO" \
        -e catkin_DIR=$SRC/catkin \
        -S -I -I.build
}

buildpkg () {
    pkg=$1
    echo "-------------" $pkg
    /bin/rm -rf $TOP/build/$pkg
    mkdir -p $TOP/build/$pkg
    cd $TOP/build/$pkg
    #cmake $SRC/$pkg -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_PREFIX_PATH=$PREFIX
    #make help
    #make $pkg-dsc
    #make $pkg-install

    cd $SRC/$pkg
    rm -f *${pkg}*_*.deb *${pkg}*_*.changes *${pkg}*_*.tar.gz *${pkg}*_*.dsc

    for d in $DISTROS
    do

        debuildit $1 $d
    done
}

instpkg () {
    pkg=$1
    dput ppa:straszheim/ros $SRC/${pkg}_*_source.changes
    # dpkg --contents $SRC/$pkg.deb
    # sudo dpkg -i $SRC/$pkg.deb
    /bin/echo
}

PKGNAMES="roscpp-core ros-std-msgs ros-fuerte-gencpp ros-fuerte-genmsg catkin"
# for pkg in $PKGNAMES
# do
#     sudo aptitude purge -y $pkg
# done 

#buildpkg gencpp
#dpkg --contents $SRC/ros-fuerte-gencpp_0.0.1_all.deb


fullbuild () {
    buildpkg catkin
    # instpkg catkin
}

meh() {
    buildpkg genmsg
    instpkg ${SERIES}genmsg
    buildpkg gencpp
    instpkg ${SERIES}gencpp
    buildpkg std_msgs
    instpkg ${SERIES}std-msgs
    buildpkg roscpp_core
    instpkg ${SERIES}roscpp-core
    buildpkg common_msgs
    instpkg ${SERIES}common-msgs
    buildpkg catkin_test
    instpkg ${SERIES}catkin-test
}

clean


fullbuild

# fullbuild


#for pkg in catkin genmsg gencpp std_msgs



