from __future__ import print_function
import os
import shutil


def _symlink_or_copy(src, dst):
    """
    Creates a symlink at dst to src, or if not possible, attempts to copy.
    """
    if not os.path.exists(src):
        return False

    # update relative source to be relative from destination
    if not os.path.isabs(src):
        src = os.path.relpath(src, os.path.dirname(dst))

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
    return True


def init_workspace(workspace_dir):
    """
    Create a toplevel CMakeLists.txt in the root of a workspace.

    The toplevel.cmake file is looked up either in the catkin
    workspaces contained in the CMAKE_PREFIX_PATH or relative to this
    file.  Then it tries to create a symlink first and if that fails
    copies the file.

    It installs ``manifest.xml`` to ``share/${PROJECT_NAME}``.

    .. note:: The symlink is absolute when catkin is found via the
    environment (since that indicates a different workspace and it
    may change relative location to the workspace referenced as a
    parameter). The symlink is relative when the toplevel.cmake of
    this catkin instance is used since it is part of the
    to-be-initialized workspace.

    :param workspace_dir: the path to the workspace where the
    CMakeLists.txt should be created
    :type workspace_dir: string
    """
    # verify that destination file does not exist
    dst = os.path.join(workspace_dir, 'CMakeLists.txt')
    if os.path.exists(dst):
        raise RuntimeError('File "%s" already exists' % dst)

    # get all cmake prefix paths
    env_name = 'CMAKE_PREFIX_PATH'
    paths = [path for path in os.environ[env_name].split(os.pathsep)] if env_name in os.environ and os.environ[env_name] != '' else []
    # remove non-workspace paths
    workspaces = [path for path in paths if os.path.exists(os.path.join(path, '.CATKIN_WORKSPACE'))]

    # search for toplevel file in all workspaces
    checked = []
    for workspace in workspaces:
        filename = os.path.join(workspace, '.CATKIN_WORKSPACE')
        data = ''
        with open(filename) as f:
            data = f.read()
        if data == '':
            # try from install space
            src = os.path.join(workspace, 'share', 'catkin', 'cmake', 'toplevel.cmake')
            success = _symlink_or_copy(src, dst)
            if success:
                return
            else:
                checked.append(src)
        else:
            # try from all source spaces
            for source_path in data.split(';'):
                src = os.path.join(source_path, 'catkin', 'cmake', 'toplevel.cmake')
                success = _symlink_or_copy(src, dst)
                if success:
                    return
                else:
                    checked.append(src)

    # try to find toplevel file in relative locations
    relative_cmake_paths = []
    # when catkin is in source space
    relative_cmake_paths.append(os.path.join('..', '..', 'cmake'))
    # when catkin is installed (with Python code in lib/pythonX.Y/[dist|site]-packages)
    relative_cmake_paths.append(os.path.join('..', '..', '..', '..', 'share', 'catkin', 'cmake'))
    # when catkin is installed (with Python code in lib/site-packages)
    relative_cmake_paths.append(os.path.join('..', '..', '..', 'share', 'catkin', 'cmake'))
    for relative_cmake_path in relative_cmake_paths:
        src = os.path.relpath(os.path.join(os.path.dirname(__file__), relative_cmake_path, 'toplevel.cmake'))
        if os.path.exists(src):
            success = _symlink_or_copy(src, dst)
            if success:
                return
        else:
            checked.append(src)

    raise RuntimeError('Could neither find file "toplevel.cmake" in any workspace nor relative, checked the following paths:\n%s' % ('\n'.join(checked)))
