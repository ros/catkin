#!/bin/bash -ex
output_dir=`pwd`/export
mkdir -p $output_dir
cd $output_dir
wget -o log.txt -r http://50.28.27.175/repos/building/pool/main -nd --accept '*.dsc'
