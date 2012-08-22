from __future__ import print_function
import os
import shutil


def init_workspace(workspace_dir):
    env_name = 'CATKIN_WORKSPACES'
    workspaces = os.environ[env_name].split(';') if env_name in os.environ and os.environ[env_name] != '' else []

    # search for file in all workspaces
    checked = []
    for workspace in workspaces:
        parts = workspace.split(':')
        if len(parts) > 1:
            src = os.path.join(parts[1], 'catkin', 'cmake', 'toplevel.cmake')
        else:
            src = os.path.join(workspace, 'share', 'catkin', 'cmake', 'toplevel.cmake')
        # skip this workspace if file does not exist
        if not os.path.exists(src):
            checked.append(src)
            continue
        # symlink/copy file
        dst = os.path.join(workspace_dir, 'CMakeLists.txt')
        try:
            os.symlink(src, dst)
            print('Creating symlink "%s" pointing to "%s"' % (dst, src))
        except Exception:
            try:
                shutil.copyfile(src, dst)
                print('Copying file from "%s" to "%s"' % (src, dst))
            except:
                raise RuntimeError('Could neither symlink nor copy file "%s" to "%s"' % (src, dst))
        return

    raise RuntimeError('Could not find file "catkin/cmake/toplevel.cmake" in any workspace, checked the following paths:\n%s' % ('\n'.join(checked)))
