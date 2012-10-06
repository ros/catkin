from __future__ import print_function
import os
from catkin.workspace import get_source_paths, get_workspaces

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
    valid_global_search_dirs = set(['bin', 'etc', 'include', 'lib', 'share'])
    valid_project_search_dirs = set(['etc', 'include', 'libexec', 'share'])

    # make search folders a list
    search_dirs = list(search_dirs or [])

    # determine valid search folders
    all_valid_search_dirs = valid_global_search_dirs.union(
        valid_project_search_dirs)

    diff_dirs = search_dirs.difference(all_valid_search_dirs)
    if len(diff_dirs) > 0:
        raise RuntimeError('Unsupported search folders: ' +
                           ', '.join(['"%s"' % i for i in diff_dirs]))
    valid_search_dirs = (valid_global_search_dirs
                         if project is None
                         else valid_project_search_dirs)

    diff_dirs = search_dirs.difference(valid_search_dirs)
    if len(diff_dirs) > 0:
        msg = 'Searching %s a project can not be combined with the search folders:' % ('without' if project is None else 'for')
        raise RuntimeError(msg + ', '.join(['"%s"' % i for i in diff_dirs]))

    if search_dirs is None:
        search_dirs = valid_search_dirs

    # collect candidate paths
    paths = []
    workspaces = get_workspaces()
    for workspace in workspaces:

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
                source_paths = get_source_paths(workspace)
                for source_path in source_paths:
                    p = os.path.join(source_path, project)
                    if path is not None:
                        p = os.path.join(p, path)
                    paths.append(p)

    # find all existing candidates
    existing = [p for p in paths if os.path.exists(p)]
    return (existing, paths)
