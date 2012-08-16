@@echo off
REM generated from catkin/cmake/em/setup.bat.em

# remember type of shell if not already set
if "%CATKIN_SHELL%" NEQ "" (
    set CATKIN_SHELL=bat
)

REM source setup.SHELL from parent workspace (if any)
@{
import os
parent_path = None
if 'CATKIN_WORKSPACES' in os.environ and os.environ['CATKIN_WORKSPACES'] != '':
    parent_path = os.environ['CATKIN_WORKSPACES'].split(';')[0].split(':')[0]
    print('. %s/setup.$CATKIN_SHELL' % parent_path)
}@

REM python function to prepend a value to an environment variable if it not already included (outputs the prefix if it needs to be prepended)
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

REM prepend current workspace to CATKIN_WORKSPACES
CATKIN_WORKSPACES_PREFIX=`python -c "$PYTHON_CODE_PREPENDING_TO_ENV" "@CURRENT_WORKSPACE" ";" "CATKIN_WORKSPACES"`
export CATKIN_WORKSPACES="${CATKIN_WORKSPACES_PREFIX}${CATKIN_WORKSPACES}"

REM define base dir for further variables (based on current workspace)
BASE_DIR="@(CURRENT_WORKSPACE.split(':')[0])"

REM prepend bin folder of workspace to PATH
PATH_PREFIX=`python -c "$PYTHON_CODE_PREPENDING_TO_ENV" "$BASE_DIR/bin" ";" "PATH"`
export PATH="${PATH_PREFIX}${PATH}"

REM prepend lib/pythonX.Y/..-packages folder of workspace to PYTHONPATH
REM the path suffix is calculated in catkin/cmake/python.cmake
PYTHONPATH_PREFIX=`python -c "$PYTHON_CODE_PREPENDING_TO_ENV" "$BASE_DIR/@PYTHON_INSTALL_DIR" ";" "PYTHONPATH"`
export PYTHONPATH="${PYTHONPATH_PREFIX}${PYTHONPATH}"

REM prepend include folder of workspace to CPATH
REM CPATH_PREFIX=`python -c "$PYTHON_CODE_PREPENDING_TO_ENV" "$BASE_DIR/include" ";" "CPATH"`
REM export CPATH="${CPATH_PREFIX}${CPATH}"

REM prepend lib folder of workspace to LD_LIBRARY_PATH
LD_LIBRARYPATH_PREFIX=`python -c "$PYTHON_CODE_PREPENDING_TO_ENV" "$BASE_DIR/lib" ";" "LD_LIBRARY_PATH"`
export LD_LIBRARY_PATH="${LD_LIBRARYPATH_PREFIX}${LD_LIBRARY_PATH}"

REM prepend folder of workspace (cmake-subfolder for build-space) to CMAKE_PREFIX_PATH
CMAKE_PREFIXPATH_PREFIX=`python -c "$PYTHON_CODE_PREPENDING_TO_ENV" "$BASE_DIR" ";" "CMAKE_PREFIX_PATH"`
export CMAKE_PREFIX_PATH="${CMAKE_PREFIXPATH_PREFIX}${CMAKE_PREFIX_PATH}"

REM prepend lib/pkgconfig folder of workspace to PKG_CONFIG_PATH
PKG_CONFIGPATH_PREFIX=`python -c "$PYTHON_CODE_PREPENDING_TO_ENV" "$BASE_DIR/lib/pkgconfig" ";" "PKG_CONFIG_PATH"`
export PKG_CONFIG_PATH="${PKG_CONFIGPATH_PREFIX}${PKG_CONFIG_PATH}"

REM run all environment hooks of workspace
for %%envfile in ("$BASE_DIR/etc/catkin/profile.d/*.$CATKIN_SHELL") do (
    call %%envfile
)
