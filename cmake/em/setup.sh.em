#!/usr/bin/env sh
# generated from catkin/cmake/em/setup.sh.em

# Sets various the enviroment variables and sources additional enviroment hooks.
# The output depends on the CMAKE_PREFIX_PATH and the path of this workspace.
# It tries it's best to undo changes from a previously sourced setup file before.
# Supported command line options:
# --extend: skips the undoing of changes from a previously sourced setup file
# Supported enviroment variables:
# _CATKIN_SETUP_NO_RECURSION: skips it's own recursion (when invoked by another setup file)

# environment at generation time
# CURRENT_CMAKE_PREFIX_PATH="@CMAKE_PREFIX_PATH"
# CURRENT_WORKSPACE="@SETUP_DIR"

# remember type of shell if not already set
if [ -z "$CATKIN_SHELL" ]; then
  CATKIN_SHELL=sh
fi

# decide which commands to perform
DO_CLEAN=1
IS_RECURSIVE=0
if [ "$_CATKIN_SETUP_NO_RECURSION" = "1" ]; then
  DO_CLEAN=0
  IS_RECURSIVE=1
elif [ $# -eq 1 -a "$1" = "--extend" ]; then
  DO_CLEAN=0
fi

# folder of workspace prepended to CMAKE_PREFIX_PATH
CMAKE_PREFIX_PATH_dir=""
# include folder of workspace prepended to CPATH
CPATH_dir=/include
# lib folder of workspace prepended to (DY)LD_LIBRARY_PATH
LD_LIBRARY_PATH_dir=/lib
# bin folder of workspace prepended to PATH
PATH_dir=/bin
# lib/pkgconfig folder of workspace prepended to PKG_CONFIG_PATH
PKG_CONFIG_PATH_dir=/lib/pkgconfig
# lib/pythonX.Y/..-packages folder of workspace prepended to PYTHONPATH
# the path suffix is calculated in catkin/cmake/python.cmake
PYTHONPATH_dir=/@PYTHON_INSTALL_DIR

# detect if running on Darwin platform
UNAME=`which uname`
UNAME=`$UNAME`
IS_DARWIN=0
if [ "$UNAME" = "Darwin" ]; then
  IS_DARWIN=1
fi

# reset environment variables by unrolling modifications based on all workspaces in CMAKE_PREFIX_PATH
# this does not cover modifications performed by environment hooks
if [ $DO_CLEAN -eq 1 ]; then
  export CPATH="$("@SETUP_DIR/setup.py" --remove --name CPATH --value "$CPATH_dir")"
  if [ $IS_DARWIN -eq 0 ]; then
    export LD_LIBRARY_PATH="$("@SETUP_DIR/setup.py" --remove --name LD_LIBRARY_PATH --value "$LD_LIBRARY_PATH_dir")"
  else
    export DYLD_LIBRARY_PATH="$("@SETUP_DIR/setup.py" --remove --name DYLD_LIBRARY_PATH --value "$LD_LIBRARY_PATH_dir")"
  fi
  export PATH="$("@SETUP_DIR/setup.py" --remove --name PATH --value "$PATH_dir")"
  export PKG_CONFIG_PATH="$("@SETUP_DIR/setup.py" --remove --name PKG_CONFIG_PATH --value "$PKG_CONFIG_PATH_dir")"
  export PYTHONPATH="$("@SETUP_DIR/setup.py" --remove --name PYTHONPATH --value "$PYTHONPATH_dir")"
  # rollback CMAKE_PREFIX_PATH last since the other rollback calls rely on this
  export CMAKE_PREFIX_PATH="$("@SETUP_DIR/setup.py" --remove --name CMAKE_PREFIX_PATH --value "$CMAKE_PREFIX_PATH_dir")"
fi

# prepend CURRENT_WORKSPACE and CURRENT_CMAKE_PREFIX_PATH to CMAKE_PREFIX_PATH
# the complete list must be set so that environment hooks in parent workspace have the full set available
if [ $IS_RECURSIVE -eq 0 ]; then
  export CMAKE_PREFIX_PATH="$("@SETUP_DIR/setup.py" --prefix --name CMAKE_PREFIX_PATH --value "@SETUP_DIR:@CMAKE_PREFIX_PATH")$CMAKE_PREFIX_PATH"
fi

# source setup.SHELL from parent workspaces
if [ $IS_RECURSIVE -eq 0 ]; then
  _CATKIN_SETUP_NO_RECURSION=1
  for workspace in $("@SETUP_DIR/setup.py" --get-reversed-workspaces --value "@SETUP_DIR"); do
    . "$workspace/setup.$CATKIN_SHELL"
  done
  _CATKIN_SETUP_NO_RECURSION=0
fi

# prepend folders of workspace to environment variables
export CPATH="$("@SETUP_DIR/setup.py" --prefix --name CPATH --value "@SETUP_DIR$CPATH_dir")$CPATH"
if [ $IS_DARWIN -eq 0 ]; then
  export LD_LIBRARY_PATH="$("@SETUP_DIR/setup.py" --prefix --name LD_LIBRARY_PATH --value "@SETUP_DIR$LD_LIBRARY_PATH_dir")$LD_LIBRARY_PATH"
else
  export DYLD_LIBRARY_PATH="$("@SETUP_DIR/setup.py" --prefix --name DYLD_LIBRARY_PATH --value "@SETUP_DIR$LD_LIBRARY_PATH_dir")$DYLD_LIBRARY_PATH"
fi
export PATH="$("@SETUP_DIR/setup.py" --prefix --name PATH --value "@SETUP_DIR$PATH_dir")$PATH"
export PKG_CONFIG_PATH="$("@SETUP_DIR/setup.py" --prefix --name PKG_CONFIG_PATH --value "@SETUP_DIR$PKG_CONFIG_PATH_dir")$PKG_CONFIG_PATH"
export PYTHONPATH="$("@SETUP_DIR/setup.py" --prefix --name PYTHONPATH --value "@SETUP_DIR$PYTHONPATH_dir")$PYTHONPATH"

# run all environment hooks of this workspace
FIND=`which find`
SORT=`which sort`
ENV_HOOKS_GENERIC=$($FIND "@SETUP_DIR/etc/catkin/profile.d" -maxdepth 1 -name "*.sh" 2>/dev/null | $SORT)
if [ "$CATKIN_SHELL" != "sh" ]; then
  ENV_HOOKS_SPECIFIC=$($FIND "@SETUP_DIR/etc/catkin/profile.d" -maxdepth 1 -name "*.$CATKIN_SHELL" 2>/dev/null | $SORT)
else
  ENV_HOOKS_SPECIFIC=
fi
for envfile in $ENV_HOOKS_GENERIC $ENV_HOOKS_SPECIFIC; do
  . "$envfile"
done
