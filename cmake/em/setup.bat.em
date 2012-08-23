@@echo off
REM generated from catkin/cmake/em/setup.bat.em

REM remember type of shell if not already set
if [%CATKIN_SHELL%]==[] (
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

REM Do not use EnableDelayedExpansion here, it messes with the != symbols.
echo from __future__ import print_function > catkin_prefix.py
echo import os >> catkin_prefix.py
echo import sys >> catkin_prefix.py
echo if len(sys.argv) != 4: >> catkin_prefix.py
echo     print("\nWrong number of arguments passed to Python code\n", file=sys.stderr) >> catkin_prefix.py
echo     exit(1) >> catkin_prefix.py
echo prepended_item, separator, env_name = sys.argv[1:] >> catkin_prefix.py
echo items = os.environ[env_name].split(separator) if env_name in os.environ and os.environ[env_name] != '' else [] >> catkin_prefix.py
echo print(prepended_item + (separator if items else '') if prepended_item not in items else '') >> catkin_prefix.py

setlocal EnableDelayedExpansion

REM prepend current workspace to CATKIN_WORKSPACES
set CATKIN_WORKSPACES_PREFIX=
for /f %%a in ('python catkin_prefix.py "@CURRENT_WORKSPACE" ";" "CATKIN_WORKSPACES"') do set CATKIN_WORKSPACES_PREFIX=!CATKIN_WORKSPACES_PREFIX!%%a
set CATKIN_WORKSPACES=%CATKIN_WORKSPACES_PREFIX%%CATKIN_WORKSPACES%

REM define base dir for further variables (based on current workspace)
set BASE_DIR=@(CURRENT_WORKSPACE.split(';')[0])

REM prepend bin folder of workspace to PATH
set PATH_PREFIX=
for /f %%a in ('python catkin_prefix.py "%BASE_DIR%/bin" ";" "PATH"') do set PATH_PREFIX=!PATH_PREFIX!%%a
set PATH=%PATH_PREFIX%%PATH%

REM prepend lib/pythonX.Y/..-packages folder of workspace to PYTHONPATH
REM the path suffix is calculated in catkin/cmake/python.cmake
set PYTHONPATH_PREFIX=
for /f %%a in ('python catkin_prefix.py "%BASE_DIR%/@PYTHON_INSTALL_DIR" ";" "PYTHONPATH"') do set PYTHONPATH_PREFIX=!PYTHONPATH_PREFIX!%%a
set PYTHONPATH=%PYTHONPATH_PREFIX%%PYTHONPATH%

REM prepend include folder of workspace to CPATH
set CPATH_PREFIX=
for /f %%a in ('python catkin_prefix.py "%BASE_DIR%/include" ";" "CPATH"') do set CPATH_PREFIX=!CPATH_PREFIX!%%a
set CPATH=%CPATH_PREFIX%%CPATH%

REM prepend lib folder of workspace to LD_LIBRARY_PATH
set LD_LIBRARYPATH_PREFIX=
for /f %%a in ('python catkin_prefix.py "%BASE_DIR%/lib" ";" "LD_LIBRARY_PATH"') do set LD_LIBRARYPATH_PREFIX=!LD_LIBRARYPATH_PREFIX!%%a
set LD_LIBRARY_PATH=%LD_LIBRARYPATH_PREFIX%%LD_LIBRARY_PATH%

REM prepend folder of workspace (cmake-subfolder for build-space) to CMAKE_PREFIX_PATH
set CMAKE_PREFIXPATH_PREFIX=
for /f %%a in ('python catkin_prefix.py "%BASE_DIR%" ";" "CMAKE_PREFIX_PATH"') do set CMAKE_PREFIXPATH_PREFIX=!CMAKE_PREFIXPATH_PREFIX!%%a
set CMAKE_PREFIX_PATH=%CMAKE_PREFIXPATH_PREFIX%%CMAKE_PREFIX_PATH%

REM prepend lib/pkgconfig folder of workspace to PKG_CONFIG_PATH
set PKG_CONFIGPATH_PREFIX=
for /f %%a in ('python catkin_prefix.py "%BASE_DIR%/lib/pkgconfig" ";" "PKG_CONFIG_PATH"') do set PKG_CONFIGPATH_PREFIX=!PKG_CONFIGPATH_PREFIX!%%a
set PKG_CONFIG_PATH=%PKG_CONFIGPATH_PREFIX%%PKG_CONFIG_PATH%

REM run all environment hooks of workspace
for %%f in ("%BASE_DIR%\etc\catkin\profile.d\*.%CATKIN_SHELL%") do (
    call %%f
)

del catkin_prefix.py

endlocal