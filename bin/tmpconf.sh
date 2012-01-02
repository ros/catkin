#!/bin/bash -e

TOP=$(cd `dirname $0` ; /bin/pwd)
. $TOP/catkin_util.sh
set -x
rm -rf tmprepo
git clone git@github.com:wg-debs/$1.git tmprepo
cd tmprepo

$TOP/git-catkin-set-upstream git@github.com:willowgarage/$1.git
git push origin catkin || /bin/true
git push origin :catkin_state || /bin/true
