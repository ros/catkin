@@echo off
REM generated from catkin/cmake/em/setup.bat.em

REM remember type of shell if not already set
if [%CATKIN_SHELL%]==[] (
  set CATKIN_SHELL=bat
)

REM called without arguments: create a clean environment, resetting all already set variables
REM   - removes all prepended items from environment variables (based on current CATKIN_WORKSPACES)
REM   - sets CATKIN_WORKSPACES to current + list of parents
REM   - calls setup from all parent workspaces with "--recursive" argument
REM   - prepends current to all variables
REM   - sources current environment hooks
REM called with "--recursive": used in recursive call from other setup
REM   - does not touch CATKIN_WORKSPACES
REM   - prepends current to all variables
REM   - sources current environment hooks
REM called with "--extend": used to extend one environment with a second one
REM   - prepends current + list of parent to existing CATKIN_WORKSPACES
REM   - calls setup from all parent workspaces with "--recursive" argument
REM   - prepends current to all variables
REM   - sources current environment hooks

REM bin folder of workspace prepended to PATH
set PATH_dir=/bin
REM lib/pythonX.Y/..-packages folder of workspace prepended to PYTHONPATH
REM the path suffix is calculated in catkin/cmake/python.cmake
set PYTHONPATH_dir=/@PYTHON_INSTALL_DIR
REM include folder of workspace prepended to CPATH
set CPATH_dir=/include
REM lib folder of workspace prepended to (DY)LD_LIBRARY_PATH
set LD_LIBRARY_PATH_dir=/lib
REM folder of workspace prepended to CMAKE_PREFIX_PATH
set CMAKE_PREFIX_PATH_dir=""
REM lib/pkgconfig folder of workspace prepended to PKG_CONFIG_PATH
set PKG_CONFIG_PATH_dir=/lib/pkgconfig
REM set the DIRNAME
for %%F in (%0) do set DIRNAME=%%~dpF
REM set the python executable
@{import sys}
set python=@(sys.executable)

REM decide which mode is requested
set RECURSIVE_MODE=0
set EXTEND_MODE=0
set NORMAL_MODE=0
if [%_CATKIN_SETUP_RECURSION%] == [1] (
  set RECURSIVE_MODE=1
) else (
  if [%1] == ["--extend"] (
    set EXTEND_MODE=1
  ) else (
    set NORMAL_MODE=1
  )
)

REM reset environment variables by unrolling modifications based on CATKIN_WORKSPACES (when called without arguments)
if [%NORMAL_MODE%] == [1] (
  for /f %%a in ('%python% %DIRNAME%\setup.py --remove --name PATH --value "%PATH_dir%"') do set PATH=%%a
  for /f %%a in ('%python% %DIRNAME%\setup.py --remove --name PYTHONPATH --value "%PYTHONPATH_dir%"') do set PYTHONPATH=%%a
  for /f %%a in ('%python% %DIRNAME%\setup.py --remove --name CPATH --value "%CPATH_dir%"') do set CPATH=%%a
  for /f %%a in ('%python% %DIRNAME%\setup.py --remove --name LD_LIBRARY_PATH --value "%LD_LIBRARY_PATH_dir%"') do set LD_LIBRARY_PATH=%%a
  for /f %%a in ('%python% %DIRNAME%\setup.py --remove --name CMAKE_PREFIX_PATH --value "%CMAKE_PREFIX_PATH_dir%"') do set CMAKE_PREFIX_PATH=%%a
  for /f %%a in ('%python% %DIRNAME%\setup.py --remove --name PKG_CONFIG_PATH --value "%PKG_CONFIG_PATH_dir%"') do set PKG_CONFIG_PATH=%%a
)


REM set CATKIN_WORKSPACES (when called without arguments)
if [%NORMAL_MODE%] == [1] (
  for /f %%a in ('%python% %DIRNAME%\setup.py --set-workspaces --value "@CURRENT_WORKSPACE;@CATKIN_WORKSPACES"') do set CATKIN_WORKSPACES=%%a
)

REM prepend current and parent workspaces to CATKIN_WORKSPACES (when called with argument --extend)
if [%EXTEND_MODE%] == [1] (
  for /f %%a in ('%python% %DIRNAME%\setup.py --set-workspaces --value "@CURRENT_WORKSPACE;@CATKIN_WORKSPACES"') do set CATKIN_WORKSPACES=%%a%CATKIN_WORKSPACES%
)

REM source setup.SHELL from parent workspaces (if any) (when called without --recursive argument)
@[if CATKIN_WORKSPACES]@
if [%EXTEND_MODE%] == [1] (
  set _CATKIN_SETUP_RECURSION=1
  for %%a in (@(' '.join(['"%s"' % ws.split(':')[0] for ws in reversed(CATKIN_WORKSPACES.split(';'))]))) do call %%a\setup.bat --recursive
  set _CATKIN_SETUP_RECURSION=0
)
@[end if]@

REM define base dir of workspace (based on CURRENT_WORKSPACE) (after sourcing the parents)
set BASE_DIR="@(CURRENT_WORKSPACE.split(':')[1].split(':')[0])"

REM prepend folders of workspace to environment variables
for /f %%a in ('%python% %DIRNAME%\setup.py --prefix --name PATH --value %BASE_DIR%%PATH_dir%') do set PATH=%%a%PATH%
for /f %%a in ('%python% %DIRNAME%\setup.py --prefix --name PYTHONPATH --value %BASE_DIR%%PYTHONPATH_dir%') do set PYTHONPATH=%%a%PYTHONPATH%
for /f %%a in ('%python% %DIRNAME%\setup.py --prefix --name CPATH --value %BASE_DIR%%CPATH_dir%') do set CPATH=%%a%CPATH%
REM for /f %%a in ('%python% %DIRNAME%\setup.py --prefix --name LD_LIBRARY_PATH --value %BASE_DIR%%LD_LIBRARY_PATH_dir%) do set LD_LIBRARY_PATH=%%a%LD_LIBRARY_PATH%
REM for /f %%a in ('%python% %DIRNAME%\setup.py --prefix --name CMAKE_PREFIX_PATH --value %BASE_DIR%%CMAKE_PREFIX_PATH_dir%) do set CMAKE_PREFIX_PATH=%%a%CMAKE_PREFIX_PATH%
REM for /f %%a in ('%python% %DIRNAME%\setup.py --prefix --name PKG_CONFIG_PATH --value %BASE_DIR%%PKG_CONFIG_PATH_dir%) do set PKG_CONFIG_PATH=%%a%PKG_CONFIG_PATH%

REM run all environment hooks of this workspace
for /r %%a in (%BASE_DIR%\etc\catkin\profile.d\*.bat) do call "%%a"
