import ast
import os
import platform
import subprocess
import sys


def generate_environment_script(env_script):
    """
    Generates script code to cache environment changes of a script.
    This code assumes that the script does nothing else than changing
    variables that contain colon separated lists of PATHs, by
    replacing or prepending.

    :param env_script: str The path to the script which changes the environment
    :returns: list script lines
    """
    code = []
    _append_header(code)

    _append_comment(code, 'based on a snapshot of the environment before and after calling the env script')
    _append_comment(code, 'it emulates the modifications of the env script.sh without recurring computations')

    # fetch current environment
    env = os.environ

    # fetch environment after calling env
    python_code = 'import os; print(os.environ)'
    output = subprocess.check_output([env_script, sys.executable, '-c', python_code])
    env_after = ast.literal_eval(output)

    # calculate added and modified environment variables
    added = {}
    modified = {}
    for key, value in env_after.items():
        if key not in env:
            added[key] = value
        elif env[key] != value:
            modified[key] = [env[key], value]

    code.append('')
    _append_comment(code, 'new environment variables')
    for key in sorted(added.keys()):
        _set_variable(code, key, added[key])

    code.append('')
    _append_comment(code, 'modified environment variables')
    for key in sorted(modified.keys()):
        (old_value, new_value) = modified[key]
        if new_value.endswith(os.pathsep + old_value):
            variable = ('$%s' if _is_not_windows() else '%%%s%%') % key
            _set_variable(code, key, new_value[:-len(old_value)] + variable)
        else:
            _set_variable(code, key, new_value)

    _append_footer(code)
    return code


def generate_static_environment_script(catkin_devel_prefix, cmake_prefix_path, python_install_dir):
    """
    Generates script code to mimic environment changes of the env script.

    :param catkin_devel_prefix: str The CMake variable CATKIN_DEVEL_PREFIX
    :param cmake_prefix_path: list The CMake variable CMAKE_PREFIX_PATH
    :param python_install_dir: str The CMake variable PYTHON_INSTALL_DIR
    :returns: list script lines
    """
    code = []
    _append_header(code)

    _append_comment(code, 'based on a snapshot of the environment at invocation time and the knowledge about how setup.sh work')
    _append_comment(code, 'it overwrites the environment with a static version without even calling the env script once')

    def prepend_env(static_env, name, value):
        items = os.environ[name].split(os.pathsep) if name in os.environ and os.environ[name] != '' else []
        values = [v for v in value.split(os.pathsep) if v != '']
        for v in values:
            if v not in items:
                items.insert(0, v)
        static_env[name] = os.pathsep.join(items)

    static_env = {}
    prepend_env(static_env, 'CMAKE_PREFIX_PATH', os.pathsep.join([catkin_devel_prefix] + cmake_prefix_path))
    prepend_env(static_env, 'CPATH', os.path.join(catkin_devel_prefix, 'include'))
    prepend_env(static_env, 'LD_LIBRARY_PATH', os.path.join(catkin_devel_prefix, 'lib'))
    prepend_env(static_env, 'PATH', os.path.join(catkin_devel_prefix, 'bin'))
    prepend_env(static_env, 'PKG_CONFIG_PATH', os.path.join(catkin_devel_prefix, 'lib', 'pkgconfig'))
    prepend_env(static_env, 'PYTHONPATH', os.path.join(catkin_devel_prefix, python_install_dir))

    code.append('')
    _append_comment(code, 'static environment variables')
    for key in sorted(static_env.keys()):
        _set_variable(code, key, static_env[key])

    _append_footer(code)
    return code


def _is_not_windows():
    return platform.system() != 'Windows'


def _append_header(code):
    if _is_not_windows():
        code.append('#!/usr/bin/env sh')
    else:
        code.append('@echo off')

    _append_comment(code, 'generated from catkin/python/catkin/environment_cache.py')
    code.append('')


def _append_footer(code):
    code.append('')
    if _is_not_windows():
        code.append('exec "$@"')
    else:
        code.append('%*')


def _append_comment(code, value):
    if _is_not_windows():
        comment_prefix = '#'
    else:
        comment_prefix = 'REM'
    code.append('%s %s' % (comment_prefix, value))


def _set_variable(code, key, value):
    if _is_not_windows():
        export_command = 'export'
    else:
        export_command = 'set'
    code.append('%s %s="%s"' % (export_command, key, value))
