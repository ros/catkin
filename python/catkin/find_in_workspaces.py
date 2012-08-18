from __future__ import print_function
import os


def find_in_workspaces(project, path=None, search_in=None):
    if path is None:
        path = ''

    # define all possible subfolders
    subfolders = {
      'bin': 'bin',
      'lib': 'lib',
      'libexec': os.path.join('lib', project),
      'share': os.path.join('share', project),
    }
    if search_in is None:
        search_in = subfolders.keys()

    env_name = 'CATKIN_WORKSPACES'
    workspaces = os.environ[env_name].split(';') if env_name in os.environ and os.environ[env_name] != '' else []

    # search for path in all workspaces
    checked = []
    for workspace in workspaces:
        parts = workspace.split(':')
        if len(parts) > 1:
            workspace = parts[1]

        # check subfolders for existance
        subfolders_exist = {}
        for key, subfolder in subfolders.items():
            subfolder = os.path.join(workspace, subfolder, '')
            subfolders_exist[key] = os.path.exists(subfolder)

        # check if project is part of this workspace
        if not subfolders_exist['libexec'] and not subfolders_exist['share']:
            continue

        # find all matching paths
        matches = []
        for key in search_in:
            if not subfolders_exist[key]:
                continue
            candidate = os.path.join(workspace, subfolders[key], path)
            checked.append(candidate)
            if os.path.exists(candidate):
                matches.append(candidate)

        if len(matches) > 1:
            raise RuntimeError('Could not find unique path "%s" for project "%s" in workspace "%s", the following paths are matching:\n%s' % (path, project, workspace, '\n'.join(matches)))
        if len(matches) == 1:
            return matches[0]

    raise RuntimeError('Could not find "%s" in any workspace, checked the following paths:\n%s' % (path, '\n'.join(checked)))
