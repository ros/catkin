#!/bin/zsh
# IT IS UNLIKELY YOU WANT TO EDIT THIS FILE BY HAND
# IF YOU WANT TO CHANGE THE ROS ENVIRONMENT VARIABLES
# EDIT "setup.sh" IN THIS DIRECTORY.

# Load the path of this particular setup.zsh                                                                                                                  
SCRIPT_PATH="$(dirname $0)";

. $SCRIPT_PATH/setup.sh

if [ -e ${ROS_ROOT}/tools/rosbash/roszsh ]; then
  . ${ROS_ROOT}/tools/rosbash/roszsh
fi
