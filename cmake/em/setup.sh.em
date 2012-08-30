#!/bin/sh
# generated from catkin/cmake/em/setup.sh.em
# CURRENT_WORKSPACE="@CURRENT_WORKSPACE"
# CATKIN_WORKSPACES="@CATKIN_WORKSPACES"

# absolute path to the folder containing this script
DIRNAME=@SETUP_DIR
# remember type of shell if not already set
if [ -z "$CATKIN_SHELL" ] ; then
  CATKIN_SHELL=sh
fi

# called without arguments: create a clean environment, resetting all already set variables
#   - removes all prepended items from environment variables (based on current CATKIN_WORKSPACES)
#   - sets CATKIN_WORKSPACES to current + list of parents
#   - calls setup from all parent workspaces with "--recursive" argument
#   - prepends current to all variables
#   - sources current environment hooks
# called with "--recursive": used in recursive call from other setup
#   - does not touch CATKIN_WORKSPACES
#   - prepends current to all variables
#   - sources current environment hooks
# called with "--extend": used to extend one environment with a second one
#   - prepends current + list of parent to existing CATKIN_WORKSPACES
#   - calls setup from all parent workspaces with "--recursive" argument
#   - prepends current to all variables
#   - sources current environment hooks

# bin folder of workspace prepended to PATH
PATH_dir=/bin
# lib/pythonX.Y/..-packages folder of workspace prepended to PYTHONPATH
# the path suffix is calculated in catkin/cmake/python.cmake
PYTHONPATH_dir=/@PYTHON_INSTALL_DIR
# include folder of workspace prepended to CPATH
CPATH_dir=/include
# lib folder of workspace prepended to (DY)LD_LIBRARY_PATH
LD_LIBRARY_PATH_dir=/lib
# folder of workspace prepended to CMAKE_PREFIX_PATH
CMAKE_PREFIX_PATH_dir=""
# lib/pkgconfig folder of workspace prepended to PKG_CONFIG_PATH
PKG_CONFIG_PATH_dir=/lib/pkgconfig

# decide which mode is requested
RECURSIVE_MODE=0
EXTEND_MODE=0
NORMAL_MODE=0
if [ "$_CATKIN_SETUP_RECURSION" = "1" ]; then
  RECURSIVE_MODE=1
elif [ $# -eq 1 -a "$1" = "--extend" ]; then
  EXTEND_MODE=1
else
  NORMAL_MODE=1
fi

# detect if running on Darwin platform
UNAME=`which uname`
UNAME=`$UNAME`
IS_DARWIN=0
if [ "$UNAME" = "Darwin" ]; then
  IS_DARWIN=1
fi

# reset environment variables by unrolling modifications based on CATKIN_WORKSPACES (when called without arguments)
if [ $NORMAL_MODE -eq 1 ]; then
  export PATH=$($DIRNAME/setup.py --remove --name PATH --value "$PATH_dir")
  export PYTHONPATH=$($DIRNAME/setup.py --remove --name PYTHONPATH --value "$PYTHONPATH_dir")
  export CPATH=$($DIRNAME/setup.py --remove --name CPATH --value "$CPATH_dir")
  if [ $IS_DARWIN -eq 0 ]; then
    export LD_LIBRARY_PATH=$($DIRNAME/setup.py --remove --name LD_LIBRARY_PATH --value "$LD_LIBRARY_PATH_dir")
  else
    export DYLD_LIBRARY_PATH=$($DIRNAME/setup.py --remove --name DYLD_LIBRARY_PATH --value "$LD_LIBRARY_PATH_dir")
  fi
  export CMAKE_PREFIX_PATH=$($DIRNAME/setup.py --remove --name CMAKE_PREFIX_PATH --value "$CMAKE_PREFIX_PATH_dir")
  export PKG_CONFIG_PATH=$($DIRNAME/setup.py --remove --name PKG_CONFIG_PATH --value "$PKG_CONFIG_PATH_dir")
fi

# set CATKIN_WORKSPACES (when called without arguments)
if [ $NORMAL_MODE -eq 1 ]; then
  export CATKIN_WORKSPACES=`$DIRNAME/setup.py --set-workspaces --value "@CURRENT_WORKSPACE;@CATKIN_WORKSPACES"`
fi

# prepend current and parent workspaces to CATKIN_WORKSPACES (when called with argument --extend)
if [ $EXTEND_MODE -eq 1 ]; then
  export CATKIN_WORKSPACES=`$DIRNAME/setup.py --prefix-workspaces --value "@CURRENT_WORKSPACE;@CATKIN_WORKSPACES"`$CATKIN_WORKSPACES
fi

# source setup.SHELL from parent workspaces (if any) (when called without --recursive argument)
@[if CATKIN_WORKSPACES]@
if [ $RECURSIVE_MODE -eq 0 ]; then
  _CATKIN_SETUP_RECURSION=1
  for workspace in @(' '.join(['"%s"' % ws.split(':')[0] for ws in reversed(CATKIN_WORKSPACES.split(';'))]))
    do
      . $workspace/setup.$CATKIN_SHELL --recursive
    done
  _CATKIN_SETUP_RECURSION=0
fi
@[end if]@

# define base dir of workspace (based on CURRENT_WORKSPACE) (after sourcing the parents)
BASE_DIR="@(CURRENT_WORKSPACE.split(':')[0])"

# prepend folders of workspace to environment variables
export PATH=$($DIRNAME/setup.py --prefix --name PATH --value $BASE_DIR$PATH_dir)$PATH
export PYTHONPATH=$($DIRNAME/setup.py --prefix --name PYTHONPATH --value $BASE_DIR$PYTHONPATH_dir)$PYTHONPATH
export CPATH=$($DIRNAME/setup.py --prefix --name CPATH --value $BASE_DIR$CPATH_dir)$CPATH
if [ $IS_DARWIN -eq 0 ]; then
  export LD_LIBRARY_PATH=$($DIRNAME/setup.py --prefix --name LD_LIBRARY_PATH --value $BASE_DIR$LD_LIBRARY_PATH_dir)$LD_LIBRARY_PATH
else
  export DYLD_LIBRARY_PATH=$($DIRNAME/setup.py --prefix --name DYLD_LIBRARY_PATH --value $BASE_DIR$LD_LIBRARY_PATH_dir)$DYLD_LIBRARY_PATH
fi
export CMAKE_PREFIX_PATH=$($DIRNAME/setup.py --prefix --name CMAKE_PREFIX_PATH --value $BASE_DIR$CMAKE_PREFIX_PATH_dir)$CMAKE_PREFIX_PATH
export PKG_CONFIG_PATH=$($DIRNAME/setup.py --prefix --name PKG_CONFIG_PATH --value $BASE_DIR$PKG_CONFIG_PATH_dir)$PKG_CONFIG_PATH

# run all environment hooks of this workspace
FIND=`which find`
for envfile in `$FIND "$BASE_DIR/etc/catkin/profile.d" -maxdepth 1 -name "*.sh" 2>/dev/null`
do
  . $envfile
done
if [ "$CATKIN_SHELL" != "sh" ]; then
  for envfile in `$FIND "$BASE_DIR/etc/catkin/profile.d" -maxdepth 1 -name "*.$CATKIN_SHELL" 2>/dev/null`
  do
    . $envfile
  done
fi
