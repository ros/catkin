#!/usr/bin/env python

from __future__ import print_function
import argparse
import ast
import os
import platform
import subprocess

# it takes a snapshot of the environment before and after calling env.sh
# the shell assignments necessary to achieve the same delta are printed

parser = argparse.ArgumentParser(description='Generates shell code which emulates the effect of the env script.')
parser.add_argument('env', help='The env.sh|bat script')
parser.add_argument('--python', default='python', help='The Python executable (default: python)')
args = parser.parse_args()

# fetch environment before calling env
env_before = os.environ

# fetch environment after calling env
python_code = 'import os; print(os.environ)'
output = subprocess.check_output([args.env, args.python, '-c', python_code])
env_after = ast.literal_eval(output)

# calculate added and modified environment variables
added = {}
modified = {}
for key, value in env_after.items():
    if key not in env_before:
        added[key] = value
    elif env_before[key] != value:
        modified[key] = [env_before[key], value]

non_windows = platform.system() != 'Windows'
if non_windows:
    print('#!/usr/bin/env sh')
else:
    print('@echo off')

print('# generated from catkin/cmake/env_caching.py')
print('# based on a snapshot of the environment before and after calling the env script')
print('# it emulates the modifications of the env script.sh without recurring computations')

print('# new environment variables')
for key in sorted(added.keys()):
    print('export %s="%s"' % (key, added[key]))

print('# modified environment variables')
for key in sorted(modified.keys()):
    (old_value, new_value) = modified[key]
    if new_value.endswith(old_value):
        variable = ('$%s' if non_windows else '%%%s%%') % key
        print('export %s="%s%s"' % (key, new_value[:-len(old_value)], variable))
    else:
        print('export %s="%s"' % (key, new_value))

if non_windows:
    print('exec "$@"')
else:
    print('%*')
