# generated from catkin/cmake/em/setup.sh.em

# remember type of shell if not already set
if [ -z "$CATKIN_SHELL" ] ; then
    CATKIN_SHELL=sh
fi

# source setup.SHELL from parent workspace (if any)
@{
import os
parent_path = None
if 'CATKIN_WORKSPACES' in os.environ and os.environ['CATKIN_WORKSPACES'] != '':
    parent_path = os.environ['CATKIN_WORKSPACES'].split(';')[0].split(':')[0]
    print('. %s/setup.$CATKIN_SHELL' % parent_path)
}@

# python function to prepend a value to an environment variable if it not already included (outputs the prefix if it needs to be prepended)
PYTHON_CODE_PREPENDING_TO_ENV=$(cat <<EOF
from __future__ import print_function
import os
import sys
if len(sys.argv) != 4:
    print("\nWrong number of arguments passed to Python code\n", file=sys.stderr)
    exit(1)
prepended_item, separator, env_name = sys.argv[1:]
items = os.environ[env_name].split(separator) if env_name in os.environ and os.environ[env_name] != '' else []
print(prepended_item + (separator if items else '') if prepended_item not in items else '')
EOF
)

# prepend current workspace to CATKIN_WORKSPACES
CATKIN_WORKSPACES_PREFIX=`python -c "$PYTHON_CODE_PREPENDING_TO_ENV" "@CURRENT_WORKSPACE" ";" "CATKIN_WORKSPACES"`
export CATKIN_WORKSPACES="${CATKIN_WORKSPACES_PREFIX}${CATKIN_WORKSPACES}"

# define base dir for further variables (based on current workspace)
BASE_DIR="@(CURRENT_WORKSPACE.split(':')[0])"

# prepend bin folder of workspace to PATH
PATH_PREFIX=`python -c "$PYTHON_CODE_PREPENDING_TO_ENV" "$BASE_DIR/bin" ":" "PATH"`
export PATH="${PATH_PREFIX}${PATH}"

# prepend lib/pythonX.Y/..-packages folder of workspace to PYTHONPATH
# the path suffix is calculated in catkin/cmake/python.cmake
PYTHONPATH_PREFIX=`python -c "$PYTHON_CODE_PREPENDING_TO_ENV" "$BASE_DIR/@PYTHON_INSTALL_DIR" ":" "PYTHONPATH"`
export PYTHONPATH="${PYTHONPATH_PREFIX}${PYTHONPATH}"

# prepend include folder of workspace to CPATH
#CPATH_PREFIX=`python -c "$PYTHON_CODE_PREPENDING_TO_ENV" "$BASE_DIR/include" ":" "CPATH"`
#export CPATH="${CPATH_PREFIX}${CPATH}"

# prepend lib folder of workspace to (DY)LD_LIBRARY_PATH
UNAME=`which uname`
if [ `$UNAME` = Darwin ]; then
    DYLD_LIBRARYPATH_PREFIX=`python -c "$PYTHON_CODE_PREPENDING_TO_ENV" "$BASE_DIR/lib" ":" "DYLD_LIBRARY_PATH"`
    export DYLD_LIBRARY_PATH="${DYLD_LIBRARYPATH_PREFIX}${DYLD_LIBRARY_PATH}"
else
    LD_LIBRARYPATH_PREFIX=`python -c "$PYTHON_CODE_PREPENDING_TO_ENV" "$BASE_DIR/lib" ":" "LD_LIBRARY_PATH"`
    export LD_LIBRARY_PATH="${LD_LIBRARYPATH_PREFIX}${LD_LIBRARY_PATH}"
fi

# prepend folder of workspace (cmake-subfolder for build-space) to CMAKE_PREFIX_PATH
CMAKE_PREFIXPATH_PREFIX=`python -c "$PYTHON_CODE_PREPENDING_TO_ENV" "$BASE_DIR" ":" "CMAKE_PREFIX_PATH"`
export CMAKE_PREFIX_PATH="${CMAKE_PREFIXPATH_PREFIX}${CMAKE_PREFIX_PATH}"

# prepend lib/pkgconfig folder of workspace to PKG_CONFIG_PATH
PKG_CONFIGPATH_PREFIX=`python -c "$PYTHON_CODE_PREPENDING_TO_ENV" "$BASE_DIR/lib/pkgconfig" ":" "PKG_CONFIG_PATH"`
export PKG_CONFIG_PATH="${PKG_CONFIGPATH_PREFIX}${PKG_CONFIG_PATH}"

# run all environment hooks of workspace
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
