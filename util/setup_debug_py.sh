#!/bin/sh -ex

/bin/echo "Setting up debug python/boost environment in opt"

TOP=$(/bin/pwd)
DOWNLOADS=$TOP/downloads

PREFIX=$TOP/opt
export PATH=$PREFIX/bin:$PATH
export LD_LIBRARY_PATH=$PREFIX/lib:$LD_LIBRARY_PATH

mkdir -p $PREFIX

wget -c http://www.python.org/ftp/python/2.7/Python-2.7.tgz
rm -rf Python-2.7
tar xvzf Python-2.7.tgz
cd Python-2.7
./configure --prefix=$PREFIX \
    --with-valgrind --without-pymalloc --enable-shared \
    --with-pydebug --with-system-expat --with-system-ffi

make
make install

cd $TOP
wget -c http://pyyaml.org/download/pyyaml/PyYAML-3.10.tar.gz
rm -rf PyYAML-3.10
tar xvzf PyYAML-3.10.tar.gz
cd PyYAML-3.10
$PREFIX/bin/python setup.py install --prefix=$PREFIX


cd $TOP
wget http://downloads.sourceforge.net/project/boost/boost/1.46.1/boost_1_46_1.tar.gz
rm -rf boost_1_46_1
tar xvzf boost_1_46_1.tar.gz
cd boost_1_46_1
./bootstrap --with-python-root=$PREFIX --prefix=$PREFIX
./bjam install --prefix=$PREFIX

cmake -DBOOST_ROOT=$PREFIX -DBoost_NO_SYSTEM_PATHS=TRUE



