from __future__ import print_function
import os
import shutil


def _symlink_toplevel_cmake(src, dst, checked):

    # check if toplevel file exists
    src = os.path.join(src, 'toplevel.cmake')
    if not os.path.exists(src):
        checked.append(src)
        return False

    if os.path.isabs(src):
        # we prefer relative symlinks so that we can move the workspace
        src_rel = os.path.relpath(src, dst)


    # try to symlink file
    dst = os.path.join(dst, 'CMakeLists.txt')
    try:
        os.symlink(src_rel, dst)
        print('Creating symlink "%s" pointing to "%s"' % (dst, src_rel))
    except Exception:
        # try to copy file
        try:
            shutil.copyfile(src, dst)
            print('Copying file from "%s" to "%s"' % (src, dst))
        except:
            raise RuntimeError('Could neither symlink nor copy file "%s" to "%s"' % (src, dst))
    return True


def init_workspace(workspace_dir):
    env_name = 'CATKIN_WORKSPACES'
    workspaces = os.environ[env_name].split(';') if env_name in os.environ and os.environ[env_name] != '' else []

    # search for toplevel file in all workspaces
    checked = []
    for workspace in workspaces:
        parts = workspace.split(':')
        if len(parts) > 1:
            src = os.path.join(parts[1], 'catkin', 'cmake')
        else:
            src = os.path.join(workspace, 'share', 'catkin', 'cmake')
        success = _symlink_toplevel_cmake(src, workspace_dir, checked)
        if success:
            return

    # search for toplevel file in relative location
    src = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'cmake'))
    success = _symlink_toplevel_cmake(src, workspace_dir, checked)
    if success:
        return

    raise RuntimeError('Could not find file "catkin/cmake/toplevel.cmake" in any workspace, checked the following paths:\n%s' % ('\n'.join(checked)))
