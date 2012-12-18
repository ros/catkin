from __future__ import print_function
import os
import subprocess


def get_repository_type(path):
    for vcs_type in ['bzr', 'git', 'hg', 'svn']:
        if os.path.isdir(os.path.join(path, '.%s' % vcs_type)):
            return vcs_type
    return None


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
