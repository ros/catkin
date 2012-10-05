@{
from __future__ import print_function
import ast
import os
import platform
import subprocess

non_windows = platform.system() != 'Windows'
if non_windows:
    print('#!/usr/bin/env sh')
    export_command = 'export'
else:
    print('@echo off')
    export_command = 'set'
}@
# generated from catkin/cmake/em/env_cached.em

@[if not CATKIN_STATIC_ENV]@
# based on a snapshot of the environment before and after calling the env script
# it emulates the modifications of the env script.sh without recurring computations
@{
# fetch current environment
env = os.environ

# fetch environment after calling env
python_code = 'import os; print(os.environ)'
output = subprocess.check_output([ENV_SCRIPT, PYTHON_EXECUTABLE, '-c', python_code])
env_after = ast.literal_eval(output)

# calculate added and modified environment variables
added = {}
modified = {}
for key, value in env_after.items():
    if key not in env:
        added[key] = value
    elif env[key] != value:
        modified[key] = [env[key], value]
}@

# new environment variables
@[for key in sorted(added.keys())]@
@export_command @key="@(added[key])"
@[end for]@

# modified environment variables
@{
for key in sorted(modified.keys()):
    (old_value, new_value) = modified[key]
    if new_value.endswith(old_value):
        variable = ('$%s' if non_windows else '%%%s%%') % key
        print('%s %s="%s%s"' % (export_command, key, new_value[:-len(old_value)], variable))
    else:
        print('%s %s="%s"' % (export_command, key, new_value))
}@
@[else]@
# based on a snapshot of the environment at cmake time and the knowledge about how setup.sh work
# it overwrites the environment with a static version without even calling the env script once
@{
def prepend_env(static_env, name, value):
    items = os.environ[name].split(os.pathsep) if name in os.environ and os.environ[name] != '' else []
    values = [v for v in value.split(os.pathsep) if v != '']
    for v in values:
        if v not in items:
            items.insert(0, v)
    static_env[name] = os.pathsep.join(items)

static_env = {}
prepend_env(static_env, 'CMAKE_PREFIX_PATH', os.pathsep.join([CATKIN_BUILD_PREFIX] + CMAKE_PREFIX_PATH))
prepend_env(static_env, 'CPATH', os.path.join(CATKIN_BUILD_PREFIX, 'include'))
prepend_env(static_env, 'LD_LIBRARY_PATH', os.path.join(CATKIN_BUILD_PREFIX, CATKIN_GLOBAL_LIB_DESTINATION))
prepend_env(static_env, 'PATH', os.path.join(CATKIN_BUILD_PREFIX, CATKIN_GLOBAL_BIN_DESTINATION))
prepend_env(static_env, 'PKG_CONFIG_PATH', os.path.join(CATKIN_BUILD_PREFIX, 'lib', 'pkgconfig'))
prepend_env(static_env, 'PYTHONPATH', os.path.join(CATKIN_BUILD_PREFIX, PYTHON_INSTALL_DIR))
}@

# static environment variables
@[for key in sorted(static_env.keys())]@
@export_command @key="@(static_env[key])"
@[end for]@
@[end if]@

@[if non_windows]@
exec "$@@"
@[else]@
%*
@[end if]@
