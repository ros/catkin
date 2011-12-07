#!/bin/bash
# IT IS UNLIKELY YOU WANT TO EDIT THIS FILE BY HAND
# IF YOU WANT TO CHANGE THE ROS ENVIRONMENT VARIABLES
# EDIT "setup.sh" IN THIS DIRECTORY.

# Load the path of this particular setup.bash                                                                                                                  

SCRIPT_PATH="${BASH_SOURCE[0]}";
if([ -h "${SCRIPT_PATH}" ]) then
  while([ -h "${SCRIPT_PATH}" ]) do SCRIPT_PATH=`readlink "${SCRIPT_PATH}"`; done
fi
pushd . > /dev/null
cd `dirname ${SCRIPT_PATH}` > /dev/null
SCRIPT_PATH=`pwd`;
popd  > /dev/null


. $SCRIPT_PATH/setup.sh

if [ -e ${ROS_ROOT}/tools/rosbash/rosbash ]; then
  . ${ROS_ROOT}/tools/rosbash/rosbash
fi
