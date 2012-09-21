from __future__ import print_function
import os
import subprocess


def get_repository_type(path):
    for vcs_type in ['bzr', 'git', 'hg', 'svn']:
        if os.path.isdir(os.path.join(path, '.%s' % vcs_type)):
            return vcs_type
    return None


def find_repositories(parent_dir):
    reps = {}
    for d in [d for d in os.listdir(parent_dir) if os.path.isdir(os.path.join(parent_dir, d))]:
        vcs_type = get_repository_type(os.path.join(parent_dir, d))
        if vcs_type is not None:
            reps[d] = vcs_type
    return reps


def vcs_branch(path, vcs_type=None):
    if vcs_type is None:
        vcs_type = get_repository_type(path)
    if vcs_type in ['git', 'hg']:
        return subprocess.check_output([vcs_type, 'branch'], cwd=path)
    elif vcs_type == 'svn':
        output = subprocess.check_output(['svn', 'info'], cwd=path)
        for line in output.split(os.linesep):
            if line.startswith('URL: '):
                return line
        raise RuntimeError('Could not determine URL of svn working copy')
    else:
        raise RuntimeError('"status" command not supported for vcs type "%s"' % vcs_type)


def vcs_diff(path, vcs_type=None):
    if vcs_type is None:
        vcs_type = get_repository_type(path)
    if vcs_type in ['bzr', 'git', 'hg', 'svn']:
        return subprocess.check_output([vcs_type, 'diff'], cwd=path)
    else:
        raise RuntimeError('"diff" command not supported for vcs type "%s"' % vcs_type)


def vcs_pull(path, vcs_type=None):
    if vcs_type is None:
        vcs_type = get_repository_type(path)
    if vcs_type in ['bzr', 'git']:
        return subprocess.check_output(['git', 'pull'], cwd=path)
    elif vcs_type == 'hg':
        return subprocess.check_output(['hg', 'pull', '-u'], cwd=path)
    elif vcs_type == 'svn':
        return subprocess.check_output(['svn', 'update'], cwd=path)
    else:
        raise RuntimeError('"pull" command not supported for vcs type "%s"' % vcs_type)


def vcs_push(path, vcs_type=None):
    if vcs_type is None:
        vcs_type = get_repository_type(path)
    if vcs_type in ['bzr', 'git']:
        return subprocess.check_output(['git', 'push'], cwd=path)
    elif vcs_type == 'hg':
        return subprocess.check_output(['hg', 'push'], cwd=path)
    else:
        raise RuntimeError('"push" command not supported for vcs type "%s"' % vcs_type)


def vcs_remotes(path, vcs_type=None):
    if vcs_type is None:
        vcs_type = get_repository_type(path)
    if vcs_type == 'git':
        return subprocess.check_output(['git', 'remote', '-v'], cwd=path)
    elif vcs_type == 'hg':
        return subprocess.check_output(['hg', 'paths'], cwd=path)
    elif vcs_type == 'svn':
        output = subprocess.check_output(['svn', 'info'], cwd=path)
        for line in output.split(os.linesep):
            if line.startswith('URL: '):
                return line
        raise RuntimeError('Could not determine URL of svn working copy')
    else:
        raise RuntimeError('"remotes" command not supported for vcs type "%s"' % vcs_type)


def vcs_status(path, vcs_type=None):
    if vcs_type is None:
        vcs_type = get_repository_type(path)
    if vcs_type in ['bzr', 'git', 'hg', 'svn']:
        return subprocess.check_output([vcs_type, 'status'], cwd=path)
    else:
        raise RuntimeError('"status" command not supported for vcs type "%s"' % vcs_type)
