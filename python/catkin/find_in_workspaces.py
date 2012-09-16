from __future__ import print_function
import os


# OUT is always a list of folders
#
# IN: project=None
# OUT: foreach ws in workspaces: foreach s in search_in: cand = ws[0] + s (+ path)
#      add cand to result list if it exists
#      is not defined for s == 'libexec', bailing out
#
# IN: project=not None
# OUT: foreach ws in workspaces: foreach s in search_in: cand = ws[0] + s + project (+ path)
#      except for s == 'share', cand is a list of two paths: ws[0] + s + project (+ path) and ws[1] + project (+ path)
#      add cand to result list if it exists
#      is not defined for s in ['bin', 'lib'], bailing out
def find_in_workspaces(search_dirs=None, project=None, path=None):
    '''
    Find all paths which match the search criteria.
    All workspaces are searched in order.
    Each workspace, each search_in subfolder, the project name and the path are concatenated to define a candidate path.
    If the candidate path exists it is appended to the result list.
    :param search_dir: The list of subfolders to search in (default contains all valid values: 'bin', 'etc', 'lib', 'libexec', 'share'), ``list``
    :param project: The project name to search for (not possible with the global search_in folders 'bin' and 'lib'), ``str``
    :param path: The path, ``str``
    Note: the search might return multiple paths for a 'share' from build- and source-space.
    :returns: List of paths, ``list``
    '''

    # define valid search folders
    valid_global_search_dirs = ['bin', 'etc', 'include', 'lib', 'share']
    valid_project_search_dirs = ['etc', 'include', 'libexec', 'share']

    # make search folders a list
    if search_dirs is None:
        search_dirs = []
    if not isinstance(search_dirs, list):
        search_dirs = [search_dirs]

    # determine valid search folders
    all_valid_search_dirs = set(valid_global_search_dirs) | set(valid_project_search_dirs)
    if not all_valid_search_dirs >= set(search_dirs):
        raise RuntimeError('Unsupported search folders: %s' % ', '.join(['"%s"' % i for i in search_dirs if i not in all_valid_search_dirs]))

    valid_search_dirs = valid_global_search_dirs if project is None else valid_project_search_dirs
    if not set(valid_search_dirs) >= set(search_dirs):
        raise RuntimeError('Searching %s a project can not be combined with the search folders %s' % ('without' if project is None else 'for', ', '.join(['"%s"' % i for i in search_dirs if i not in valid_search_dirs])))
    if not search_dirs:
        search_dirs = valid_search_dirs

    # collect candidate paths
    paths = []
    env_name = 'CMAKE_PREFIX_PATH'
    # get all cmake prefix paths
    workspaces = [workspace for workspace in os.environ[env_name].split(os.pathsep)] if env_name in os.environ and os.environ[env_name] != '' else []
    # remove non-workspaces
    workspaces = [workspace for workspace in workspaces if os.path.exists(os.path.join(workspace, '.CATKIN_WORKSPACE'))]
    for workspace in workspaces:
        # determine source spaces
        filename = os.path.join(workspace, '.CATKIN_WORKSPACE')
        data = ''
        with open(filename) as f:
            data = f.read()
        source_paths = data.split(';') if data != '' else []

        for sub in search_dirs:
            # search in workspace
            p = os.path.join(workspace, sub if sub != 'libexec' else 'lib')
            if project is not None:
                p = os.path.join(p, project)
            if path is not None:
                p = os.path.join(p, path)
            paths.append(p)

            # for search in share also consider source spaces
            if project is not None and sub == 'share':
                for source_path in source_paths:
                    p = os.path.join(source_path, project)
                    if path is not None:
                        p = os.path.join(p, path)
                    paths.append(p)

    # find all existing candidates
    existing = [p for p in paths if os.path.exists(p)]
    return (existing, paths)
