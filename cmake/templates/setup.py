#!/usr/bin/env python

from __future__ import print_function
import argparse
import os
import sys


def get_reversed_workspaces(value):
    '''Return a newline separated list of workspaces in CMAKE_PREFIX_PATH in reverse order and remove any occurrences of VALUE.'''
    env_name = 'CMAKE_PREFIX_PATH'
    paths = [path for path in reversed(os.environ[env_name].split(os.pathsep))] if env_name in os.environ and os.environ[env_name] != '' else []
    paths = [path for path in paths if os.path.exists(os.path.join(path, '.CATKIN_WORKSPACE'))]
    if value is not None:
        paths = [path for path in paths if path != value]
    return '\n'.join(paths)


def prefix_env(name, value):
    '''Return the prefix to prepend VALUE to the environment variable NAME without duplicate or empty items.'''
    items = os.environ[name].split(os.pathsep) if name in os.environ and os.environ[name] != '' else []
    prefix = []
    values = [v for v in value.split(os.pathsep) if v != '']
    for v in values:
        if v not in items and v not in prefix:
            prefix.append(v)
    prefix = os.pathsep.join(prefix)
    if prefix != '' and items:
        prefix += os.pathsep
    return prefix


def remove_from_env(name, value):
    '''For each catkin workspace in CMAKE_PREFIX_PATH remove the first subfolder VALUE from env[NAME] and return the updated value of the environment variable.'''
    items = os.environ[name].split(os.pathsep) if name in os.environ and os.environ[name] != '' else []
    env_name = 'CMAKE_PREFIX_PATH'
    paths = [path for path in os.environ[env_name].split(os.pathsep)] if env_name in os.environ and os.environ[env_name] != '' else []
    for path in paths:
        if not os.path.exists(os.path.join(path, '.CATKIN_WORKSPACE')):
            continue
        try:
            items.remove(path + value)
        except ValueError:
            pass
    return os.pathsep.join(items)


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
