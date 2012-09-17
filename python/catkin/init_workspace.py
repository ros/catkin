from __future__ import print_function
import os
import shutil


def _symlink_or_copy(src, dst):
    """
    Creates a symlink at dst to src, or if not possible, attempts to copy.
    """
    # try to symlink file
    try:
        os.symlink(src, dst)
        print('Creating symlink "%s" pointing to "%s"' % (dst, src))
    except Exception as excsym:
        # try to copy file
        try:
            shutil.copyfile(src, dst)
            print('Copying file from "%s" to "%s"' % (src, dst))
        except Exception as exccp:
            raise RuntimeError('Could neither symlink nor copy file "%s" to "%s"\n%s\n%s' % (src, dst, str(excsym), str(exccp)))


def init_workspace(workspace_dir):
    """
    Create a toplevel CMakeLists.txt in the root of a workspace.

    The toplevel.cmake file is looked up either in the CATKIN_WORKSPACES
    or relative to this file.  Then it tries to create a symlink first
    and if that fails copies the file.

    It installs ``manifest.xml`` to ``share/${PROJECT_NAME}``.

    .. note:: The symlink is absolute when catkin is found outside the
    workspace_dir (since that indicates a different workspace and it
    may change relative location to the workspace referenced as a
    parameter). The symlink is relative else.

    :param workspace_dir: the path to the workspace where the
    CMakeLists.txt should be created
    :type workspace_dir: string
    """
    # verify that destination file does not exist
    dst = os.path.join(workspace_dir, 'CMakeLists.txt')
    if os.path.exists(dst):
        raise RuntimeError('File "%s" already exists' % dst)

    src_file_path = None
    # get all cmake prefix paths
    env_name = 'CMAKE_PREFIX_PATH'
    workspaces = [workspace_dir]
    if env_name in os.environ and os.environ[env_name] != '':
        workspaces.extend(os.environ[env_name].split(os.pathsep))

    # search for toplevel file in all workspaces
    checked = []
    for workspace in workspaces:
        filename = os.path.join(workspace, '.CATKIN_WORKSPACE')
        data = None
        if os.path.isfile(filename):
            with open(filename) as f:
                data = f.read()
        if data is None or data == '':
            # try from install space
            src = os.path.join(workspace, 'catkin', 'cmake', 'toplevel.cmake')
            if os.path.exists(src):
                src_file_path = src
                break
            else:
                checked.append(src)
        else:
            # try from all source spaces
            for source_path in data.split(';'):
                src = os.path.join(source_path, 'catkin', 'cmake', 'toplevel.cmake')
                if os.path.exists(src):
                    src_file_path = src
                    break
                else:
                    checked.append(src)

    if src_file_path is None:
        # try to find toplevel file in relative locations
        relative_cmake_paths = []
        # when catkin is in source space
        relative_cmake_paths.append(os.path.join('..', '..', 'cmake'))
        # when catkin is installed (with Python code in lib/pythonX.Y/[dist|site]-packages)
        relative_cmake_paths.append(os.path.join('..', '..', '..', '..', 'share', 'catkin', 'cmake'))
        # when catkin is installed (with Python code in lib/site-packages)
        relative_cmake_paths.append(os.path.join('..', '..', '..', 'share', 'catkin', 'cmake'))
        for relative_cmake_path in relative_cmake_paths:
            src = os.path.join(os.path.dirname(__file__),
                               relative_cmake_path, 'toplevel.cmake')
            if os.path.exists(src):
                src_file_path = src
                break
            else:
                checked.append(src)

    if src_file_path is None:
        raise RuntimeError('Could neither find file "toplevel.cmake" in any workspace nor relative, checked the following paths:\n%s' % ('\n'.join(checked)))

    # comparing paths is a bug minefield due to symbolic links in the path
    realpath_src = os.path.realpath(os.path.abspath(src))
    realpath_dst = os.path.realpath(os.path.abspath(workspace_dir))
    commonprefix = os.path.commonprefix([realpath_src, realpath_dst])
    if commonprefix.rstrip(os.sep) == realpath_dst:
        # catkin below workspacedir is used, use relative path
        src_file_path = os.path.relpath(src_file_path, workspace_dir)
    else:
        src_file_path = os.path.abspath(src_file_path)
    _symlink_or_copy(src_file_path, dst)
