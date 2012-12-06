#!/usr/bin/env python

'''This file provides setup.sh with commands to query catkin workspaces'''

from __future__ import print_function
import argparse
import os
import sys

CATKIN_MARKER_FILE = '.catkin'


def get_workspaces(include_fuerte=False):
    '''
    Based on CMAKE_PREFIX_PATH return all catkin workspaces

    :param include_fuerte: The flag if paths starting with '/opt/ros/fuerte' should be considered workspaces, ``bool``
    '''
    # get all cmake prefix paths
    env_name = 'CMAKE_PREFIX_PATH'
    paths = [path for path in os.environ.get(env_name, '').split(os.pathsep) if path]
    # remove non-workspace paths
    workspaces = [path for path in paths if os.path.isfile(os.path.join(path, CATKIN_MARKER_FILE)) or (include_fuerte and path.startswith('/opt/ros/fuerte'))]
    return workspaces


def get_reversed_workspaces(exclude=None):
    '''Return a newline separated list of workspaces in CMAKE_PREFIX_PATH in reverse order and remove any occurrences of EXCLUDE.'''
    paths = [p for p in reversed(get_workspaces()) if p != exclude]
    return '\n'.join(paths)


def prefix_env(name, new_paths_str):
    '''
    Return the prefix to prepend to the environment variable NAME, adding any path in NEW_PATHS_STR without creating duplicate or empty items.
    '''
    environ_paths = [i for i in os.environ.get(name, '').split(os.pathsep) if i]
    checked_paths = []
    new_paths = [v for v in new_paths_str.split(os.pathsep) if v != '']
    for path in new_paths:
        # exclude any path already in env and any path we already added
        if path not in environ_paths and path not in checked_paths:
            checked_paths.append(path)
    prefix_str = os.pathsep.join(checked_paths)
    if prefix_str != '' and environ_paths:
        prefix_str += os.pathsep
    return prefix_str


def remove_from_env(name, subfolder):
    '''
    For each catkin workspace in CMAKE_PREFIX_PATH remove the first entry from env[NAME] matching workspace + subfolder.

    :param subfolder: str '' or subfoldername that may start with '/'
    :returns: the updated value of the environment variable.
    '''
    env_paths = [path for path in os.environ.get(name, '').split(os.pathsep) if path]
    if subfolder:
        if subfolder.startswith(os.path.sep) or (os.path.altsep and subfolder.startswith(os.path.altsep)):
            subfolder = subfolder[1:]
        if subfolder.endswith(os.path.sep) or (os.path.altsep and subfolder.endswith(os.path.altsep)):
            subfolder = subfolder[:-1]
    for ws_path in get_workspaces(include_fuerte=True):
        path_to_find = os.path.join(ws_path, subfolder) if subfolder else ws_path
        path_to_remove = None
        for env_path in env_paths:
            env_path_clean = env_path[:-1] if env_path and env_path[-1] in [os.path.sep, os.path.altsep] else env_path
            if env_path_clean == path_to_find:
                path_to_remove = env_path
                break
        if path_to_remove:
            env_paths.remove(path_to_remove)
    return os.pathsep.join(env_paths)


def _parse_arguments():
    parser = argparse.ArgumentParser(description='Generates code blocks for the setup.SHELL script.')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--get-reversed-workspaces', action='store_true', help='Get workspaces based on CMAKE_PREFIX_PATH in reverse order')
    group.add_argument('--prefix', action='store_true', help='Prepend a unique value to an environment variable')
    group.add_argument('--remove', action='store_true', help='Remove the prefix for each workspace in CMAKE_PREFIX_PATH from the environment variable')
    parser.add_argument('--name', nargs='?', help='The name of the environment variable')
    parser.add_argument('--value', help='The value')
    args = parser.parse_args()

    # verify correct argument combination
    if (args.prefix or args.remove) and args.name is None:
        raise RuntimeError('Argument "--name" must be passed for "%s"' % ('--prefix' if args.prefix else '--remove'))
    if args.get_reversed_workspaces and args.name is not None:
        raise RuntimeError('Argument "--name" must not be passed for "--get-reversed-workspaces"')

    return args


if __name__ == '__main__':
    try:
        args = _parse_arguments()
    except Exception as e:
        print(e, file=sys.stderr)
        exit(1)

    # dispatch to requested operation
    if args.get_reversed_workspaces:
        print(get_reversed_workspaces(args.value))
    elif args.prefix:
        print(prefix_env(args.name, args.value))
    elif args.remove:
        print(remove_from_env(args.name, args.value))
