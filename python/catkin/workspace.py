from __future__ import print_function
import os

CATKIN_WORKSPACE_MARKER_FILE = '.CATKIN_WORKSPACE'


def get_workspaces():
    """
    Based on CMAKE_PREFIX_PATH return all catkin workspaces
    """
    # get all cmake prefix paths
    env_name = 'CMAKE_PREFIX_PATH'
    paths = [path for path in os.environ[env_name].split(os.pathsep)] if env_name in os.environ and os.environ[env_name] != '' else []
    # remove non-workspace paths
    workspaces = [path for path in paths if os.path.isfile(os.path.join(path, CATKIN_WORKSPACE_MARKER_FILE))]
    return workspaces


def get_source_paths(workspace):
    """
    reads catkin workspace files and returns the list of all declared
    source paths
    """
    # determine source spaces
    data = ''
    filename = os.path.join(workspace, CATKIN_WORKSPACE_MARKER_FILE)
    with open(filename) as f:
        data = f.read()

    if data == '':
        source_paths = []
    else:
        source_paths = data.split(';')
    return source_paths
