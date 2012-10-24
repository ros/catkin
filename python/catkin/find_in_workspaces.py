from __future__ import print_function
import os
from catkin.workspace import get_source_paths, get_workspaces
from catkin_pkg.packages import find_packages

def _get_valid_search_dirs(search_dirs, project):
    """
    compares param collection of search dirs with valid names, raises ValueError if invalid.
    maintains the order of param if any. If project is given other names are allowed than without.

    :param search_dirs: collection of foldernames (basename) to search for
    :param project: the project to search in or None
    :raises: ValueError
    """
    # define valid search folders
    valid_global_search_dirs = ['bin', 'etc', 'include', 'lib', 'share']
    valid_project_search_dirs = ['etc', 'include', 'libexec', 'share']

    valid_search_dirs = (valid_global_search_dirs
                         if project is None
                         else valid_project_search_dirs)
    if not search_dirs:
        search_dirs = valid_search_dirs
    else:
        # make search folders a list
        search_dirs = list(search_dirs)

        # determine valid search folders
        all_valid_search_dirs = set(valid_global_search_dirs).union(
            set(valid_project_search_dirs))

        # check folder name is known at all
        diff_dirs = set(search_dirs).difference(all_valid_search_dirs)
        if len(diff_dirs) > 0:
            raise ValueError('Unsupported search folders: ' +
                             ', '.join(['"%s"' % i for i in diff_dirs]))
        # check foldername works with project arg
        diff_dirs = set(search_dirs).difference(valid_search_dirs)
        if len(diff_dirs) > 0:
            msg = 'Searching %s a project can not be combined with the search folders:' % ('without' if project is None else 'for')
            raise ValueError(msg + ', '.join(['"%s"' % i for i in diff_dirs]))
    return search_dirs


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
def find_in_workspaces(search_dirs=None, project=None, path=None, _workspaces=get_workspaces()):
    '''
    Find all paths which match the search criteria.
    All workspaces are searched in order.
    Each workspace, each search_in subfolder, the project name and the path are concatenated to define a candidate path.
    If the candidate path exists it is appended to the result list.
    Note: the search might return multiple paths for a 'share' from build- and source-space.

    :param search_dir: The list of subfolders to search in (default contains all valid values: 'bin', 'etc', 'lib', 'libexec', 'share'), ``list``
    :param project: The project name to search for (optional, not possible with the global search_in folders 'bin' and 'lib'), ``str``
    :param path: The path inside the project to search for. May only be not-None if project is not None. ``str``
    :param _workspaces: (optional, used for unit tests), the list of workspaces to use.
    :raises ValueError: if search_dirs contains an invalid folder name
    :returns: List of paths, ``list``
    '''
    search_dirs = _get_valid_search_dirs(search_dirs, project)
    if path and not project:
        # should never happen, bug in client code
        raise RuntimeError('Cannot look for path without project')
    # collect candidate paths
    paths = []
    # in overlay case, only return results in upmost project
    project_found = False
    for workspace in (_workspaces or []):
        for sub in search_dirs:
            # search in workspace
            p = os.path.join(workspace, sub if sub != 'libexec' else 'lib')
            if project:
                p = os.path.join(p, project)
                if os.path.isdir(p):
                    project_found = True
                if path:
                    p = os.path.join(p, path)
            paths.append(p)

            # for search in share also consider source spaces
            if project is not None and sub == 'share':
                source_paths = get_source_paths(workspace)
                for source_path in source_paths:
                    packages = find_packages(source_path)
                    for pack_path, package in packages.items():
                        pack_path = os.path.join(source_path, pack_path)
                        if package.name == project:
                            project_found = True
                            if path is not None:
                                pack_path = os.path.join(pack_path, path)
                            paths.append(pack_path)
                            break
                    if project_found:
                        break
        if project_found:
            break
    # find all existing candidates
    existing = [p for p in paths if os.path.exists(p)]
    return (existing, paths)
