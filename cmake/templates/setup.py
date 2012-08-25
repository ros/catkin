#!/usr/bin/env python

from __future__ import print_function
import argparse
import os
import sys


def set_workspaces(value):
    '''Return the value for the environment variable CATKIN_WORKSPACES without empty or duplicate items.'''
    values = [v for v in value.split(';') if v != '']
    unique = []
    for v in values:
        if v not in unique:
            unique.append(v)
    return ';'.join(unique)


def prefix_workspaces(value):
    '''Return the prefix to prepend VALUE to the environment variable CATKIN_WORKSPACES without empty or duplicate items.'''
    name = 'CATKIN_WORKSPACES'
    items = os.environ[name].split(';') if name in os.environ and os.environ[name] != '' else []
    values = [v for v in value.split(';') if v != '']
    for v in reversed(values):
        if v not in items:
            items.insert(0, v)
    return ';'.join(items)


def prefix_env(name, value):
    '''Return the prefix to prepend VALUE to the environment variable NAME.'''
    if os.environ.get(name, '') == '':
        return value
    return '%s:' % value


def remove_from_env(name, value):
    '''Remove the first item whose value is VALUE from env[NAME] and return the updated value of the environment variable.'''
    items = os.environ[name].split(':') if name in os.environ and os.environ[name] != '' else []
    env_name = 'CATKIN_WORKSPACES'
    workspaces = [ws.split(':')[0] for ws in os.environ[env_name].split(';')] if env_name in os.environ and os.environ[env_name] != '' else []
    for workspace in workspaces:
        try:
            items.remove(workspace + value)
        except ValueError:
            pass
    return ':'.join(items)


def _parse_arguments():
    parser = argparse.ArgumentParser(description='Generates code blocks for the setup.SHELL script.')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--set-workspaces', action='store_true', help='Set current and parent workspaces in environment')
    group.add_argument('--prefix-workspaces', action='store_true', help='Prefix current and parent workspaces to environment')
    group.add_argument('--prefix', action='store_true', help='Prepend a unique value to an environment variable')
    group.add_argument('--remove', action='store_true', help='Remove a value from an environment variable')
    parser.add_argument('--name', nargs='?', help='The name of the environment variable')
    parser.add_argument('--value', help='The value')
    args = parser.parse_args()

    # verify correct argument combination
    if (args.prefix or args.remove) and args.name is None:
        raise RuntimeError('Argument "--name" must be passed for "%s"' % ('--prefix' if args.prefix else '--remove'))
    if (args.set_workspaces or args.prefix_workspaces) and args.name is not None:
        raise RuntimeError('Argument "--name" must not be passed for "%s"' % ('--set-workspaces' if args.set_workspaces else '--prefix-workspaces'))

    return args


if __name__ == "__main__":
    try:
        args = _parse_arguments()
    except Exception as e:
        print(e, file=sys.stderr)
        exit(1)

    # dispatch to requested operation
    if args.set_workspaces:
        print(set_workspaces(args.value))
    elif args.prefix_workspaces:
        print(prefix_workspaces(args.value))
    elif args.prefix:
        print(prefix_env(args.name, args.value))
    elif args.remove:
        print(remove_from_env(args.name, args.value))
