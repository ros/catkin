#!/bin/bash

TOP=$(cd `dirname $0` ; /bin/pwd)
. $TOP/catkin_util.sh

github_raw CONF wg-debs/$1/catkin/catkin.conf

/bin/echo "conf=$CONF"
