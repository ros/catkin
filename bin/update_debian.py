#!/usr/bin/env python

from __future__ import print_function
from subprocess import Popen, CalledProcessError
import datetime
import dateutil.tz
import em
import os
import subprocess
import sys
from pprint import pprint
import tarfile
import yaml

name_key = 'Catkin-ProjectName'


'''
The Debian binary package file names conform to the following convention:
<foo>_<VersionNumber>-<DebianRevisionNumber>_<DebianArchitecture>.deb

PackagePrefix is a ROS mangling, ros-electric-,ros-fuerte-,ros-X- etc...
Distribution is an ubuntu distro
Version is the upstream version
DebianInc is some number that is incremental for package maintenance
Changes is a bulleted list of changes.
'''
def parse_options():
    import argparse
    parser = argparse.ArgumentParser(description='Creates/updates a gpb from a catkin project.')
    parser.add_argument('--repo_uri', dest='repo_uri',
            help='Override the Release-Push field in the stack.yaml. A pushable git buildpackage repo uri.', default=None)
    parser.add_argument('--working', help='A scratch build path. Default: %(default)s',
                        default='tmp-%u/' % os.getpid())
    parser.add_argument(dest='upstream',
            help='The location of your sources to create an upstream snap shot from.')
    parser.add_argument('--debian_inc', dest='debian_inc', help='Bump the changelog debian number. Please enter a monotonically increasing number from the last upload.', default=0)
    parser.add_argument(dest='rosdistro', help='The ros distro. electric, fuerte, galapagos')
    parser.add_argument('--output', help='The result of source deb building will go here. For debuging purposes. Default: %(default)s', default='./sourcedeb-build')
    parser.add_argument('--distros', nargs='+',
            help='A list of debian distros. Default: %(default)s',
            default=['lucid', 'natty', 'oneiric'])
    parser.add_argument('--rosdep', help='Location of the remote rosdep database.', default='git://github.com/ros/rosdep_rules.git')
    parser.add_argument('--push', dest='push', help='Push it to your remote repo?', action='store_true')
    parser.add_argument('--first-release', dest='first_release', help='Is this your first release?', action='store_true')
    return parser.parse_args()


def call(working_dir, command, pipe=None):
    print('+ cd %s && ' % working_dir + ' '.join(command))
    process = Popen(command, stdout=pipe, stderr=pipe, cwd=working_dir)
    output, unused_err = process.communicate()
    retcode = process.poll()
    if retcode:
        raise CalledProcessError(retcode, command)
    if pipe:
        return output

def check_local_repo_exists(repo_path):
    return os.path.exists(os.path.join(repo_path, '.git'))

def update_repo(working_dir, repo_path, repo_uri, first_release):
    if check_local_repo_exists(repo_path):
        print("please start from a bare working dir::\n\trm -rf %s" % repo_path)
        sys.exit(1)
    if first_release:
        os.makedirs(repo_path)
        call(repo_path, ['git', 'init'])
        call(repo_path, ['git', 'remote', 'add', 'origin', repo_uri])
    else:
        command = ('gbp-clone', repo_uri);
        call(working_dir, command)

    command = ['git', 'config', '--add', 'remote.origin.push', '+refs/heads/*:refs/heads/*']
    call(repo_path, command)

    command = ['git', 'config', '--add', 'remote.origin.push', '+refs/tags/*:refs/tags/*']
    call(repo_path, command)

def generate_rosdep_db(working_dir, rosdep_remote, rosdistro):
    basename = os.path.splitext(os.path.basename(rosdep_remote))[0]
    repo_path = os.path.join(working_dir, basename)
    db = dict()
    if check_local_repo_exists(repo_path):
        call(repo_path, ('git', 'pull'))
    else:
        call(working_dir, ('git', 'clone', rosdep_remote))
    for subdir, dirs, files in os.walk(repo_path):
        if '.git' in subdir: continue
        if not rosdistro in subdir and subdir not in repo_path:continue
        if 'rosdep.yaml' not in files: continue
        fn = os.path.join(subdir, 'rosdep.yaml')
        print('appending to rosdep db : %s' % fn)
        db.update(yaml.load(open(fn)))
    #db = yaml.load(open(os.path.join(repo_path,'rosdep.yaml')))
    #for x in os.path
    return db

def novel_version(repo_path, version):
    tags = call(repo_path, ['git', 'tag'], pipe=subprocess.PIPE)
    if 'upstream/%s' % version in tags:
        return False
    return True

def import_orig(repo_path, upstream_tarball, version):
    if not novel_version(repo_path, version):
        print('Warning: removing previous upstream version %(version)s!'
                % locals(), file=sys.stderr)
        call(repo_path, ['git', 'tag', '-d', 'upstream/%(version)s' % locals()])
    upstream_tarball = os.path.abspath(upstream_tarball)
    if len(call(repo_path, ['git', 'diff'], pipe=subprocess.PIPE)):
        call(repo_path, ['git', 'commit', '-a', '-m', '"commit all..."'])
    call(repo_path, ['git', 'import-orig', upstream_tarball])

def make_working(working_dir):
    if not os.path.exists(working_dir):
        os.makedirs(working_dir)

def exclude(filename):
    if filename and filename[-1] == '~':
        return True;
    ignores = []
    ignores += [x == os.path.basename(filename) for x in ('.svn', '.git', '.hg')]
    ignores += [x == os.path.basename(os.path.dirname(filename)) for x in ('build',)]
    return any(ignores)

def make_tarball(upstream, tarball_name):
    tarball = tarfile.open(name=tarball_name, mode='w:gz')
    tarball.add(name=upstream, arcname=os.path.basename(upstream), recursive=True, exclude=exclude)
    tarball.close()

def sanitize_package_name(name):
    return name.replace('_', '-')

def parse_stack_yaml(upstream, rosdistro, release_push=None, args=None):
    yaml_path = os.path.join(upstream, 'stack.yaml')
    stack_yaml = yaml.load(open(yaml_path))

    if 'Catkin-ChangelogType' not in stack_yaml:
        stack_yaml['Catkin-ChangelogType'] = ''
    stack_yaml['DebianInc'] = args.debian_inc
    stack_yaml['Package'] = sanitize_package_name(stack_yaml[name_key])
    stack_yaml['ROS_DISTRO'] = rosdistro
    stack_yaml['INSTALL_PREFIX'] = '/opt/ros/%s' % rosdistro
    stack_yaml['PackagePrefix'] = 'ros-%s-' % rosdistro

    deps = stack_yaml['Depends']
    if type(deps) == str:
        stack_yaml['Depends'] = set([x.strip() for x in deps.split(',') if len(x.strip()) > 0])
    else:
        stack_yaml['Depends'] = set(deps)

    return stack_yaml

def template_dir():
    return os.path.join(os.path.dirname(__file__), 'em')

def expand(fname, stack_yaml, source_dir, dest_dir, filetype=''):
    #where normal templates live
    templatedir = template_dir()
    #the default input template file path
    ifilename = os.path.join(templatedir, fname)

    if filetype != '':
        if filetype.startswith('+'):
            ifilename = os.path.join(source_dir, filetype[1:])
        else:
            ifilename += ('.' + filetype + '.em')
    else:
        ifilename += '.em'

    print("Reading %s template from %s" % (fname, ifilename))
    file_em = open(ifilename).read()

    s = em.expand(file_em, **stack_yaml)

    ofilename = os.path.join(dest_dir, fname)
    ofilestr = open(ofilename, "w")
    print(s, file=ofilestr)
    ofilestr.close()
    if fname == 'rules':
        os.chmod(ofilename, 0755)

def find_deps(stack_yaml, rosdeb_db, distro):
    def update_deps(ubuntu_deps, dep, dep_def, distro):
        if ' ' in dep_def:
            raise RuntimeError("Corrupt rosdep with internal space: '%s', check your stack.yaml" % dep_def)

        if type(dep_def) == str:
            deps = [x.strip() for x in dep_def.split(' ') if len(x.strip()) > 0]
            ubuntu_deps.add(*deps)
        elif type(dep_def) == dict:
            if distro in dep_def:
                update_deps(ubuntu_deps, dep, dep_def[distro], distro) #recurse
            elif 'apt' in dep_def:
                if 'packages' in dep_def['apt']:
                    ubuntu_deps.add(*dep_def['apt']['packages'])
        else:
            raise RuntimeError("Poorly formatted rosdep for %s (type is %s) which isnt dict or str" % (dep, type(dep_def)))
    deps = stack_yaml['Depends']
    ubuntu_deps = set()
    for dep in deps:
        if dep not in rosdeb_db.keys():
            print("I can't find a rosdep for : %s\nAdding verbatim as an Ubuntu package name." % dep, file=sys.stderr)
            ubuntu_deps.add(dep)
        else:
            dep_def = rosdeb_db[dep]
            if 'ubuntu' in dep_def:
                update_deps(ubuntu_deps, dep, dep_def['ubuntu'], distro)
            else:
                print("I cant find an ubuntu rosdep for : %s" % dep, file=sys.stderr)
                print("I did find:", dep_def, file=sys.stderr)

    print(stack_yaml[name_key], "has the following dependencies for ubuntu %s" % distro)
    pprint(ubuntu_deps)
    return list(ubuntu_deps)

def generate_deb(stack_yaml, repo_path, stamp, debian_distro, rosdep_db):
    depends = find_deps(stack_yaml, rosdep_db, debian_distro)
    stack_yaml['DebDepends'] = depends
    stack_yaml['Distribution'] = debian_distro
    stack_yaml['Date'] = stamp.strftime('%a, %d %b %Y %T %z')
    stack_yaml['YYYY'] = stamp.strftime('%Y')

    source_dir = '.' # repo_path
    print("source_dir=%s"%source_dir)
    dest_dir = os.path.join(source_dir, 'debian')
    if not os.path.exists(dest_dir):
        os.makedirs(dest_dir)

    #create control file:
    expand('control', stack_yaml, source_dir, dest_dir)
    expand('changelog', stack_yaml, source_dir, dest_dir, filetype=stack_yaml['Catkin-ChangelogType'])
    expand('rules', stack_yaml, source_dir, dest_dir, filetype=stack_yaml['Catkin-DebRulesType'])
    expand('copyright', stack_yaml, source_dir, dest_dir, filetype=stack_yaml['Catkin-CopyrightType'])

    #compat to quiet warnings, 7 .. lucid
    ofilename = os.path.join(dest_dir, 'compat')
    ofilestr = open(ofilename, "w")
    print("7", file=ofilestr)
    ofilestr.close()

    #source format, 3.0 quilt
    if not os.path.exists(os.path.join(dest_dir, 'source')):
        os.makedirs(os.path.join(dest_dir, 'source'))
    ofilename = os.path.join(dest_dir, 'source/format')
    ofilestr = open(ofilename, "w")
    print("3.0 (quilt)", file=ofilestr)
    ofilestr.close()

def commit_debian(stack_yaml, repo_path):
    call(repo_path, ['git', 'add', 'debian'])
    message = '''+ Creating debian mods for distro: %(Distribution)s, rosdistro: %(ROS_DISTRO)s, upstream version: %(Version)s
''' % stack_yaml
    call(repo_path, ['git', 'commit', '-m', message])

def gbp_sourcedebs(stack_yaml, repo_path, output):
    tag_name = 'debian/ros_%(ROS_DISTRO)s_%(Version)s_%(Distribution)s' % stack_yaml
    tag = '--git-debian-tag=%s' % tag_name
    call(repo_path, ['git', 'buildpackage',
        '-S', '--git-export-dir=%s' % output,
        '--git-ignore-new', '--git-retag', '--git-tag', tag, '-uc', '-us'])
    return tag_name

def main(args):
    stamp = datetime.datetime.now(dateutil.tz.tzlocal())
    stack_yaml = parse_stack_yaml(args.upstream, args.rosdistro, args.repo_uri, args=args)
    pprint(stack_yaml)

    #repo_base = os.path.splitext(os.path.basename(stack_yaml['Release-Push']))[0]
    #repo_path = os.path.join(args.working, repo_base)

    make_working(args.working)

    rosdep_db = generate_rosdep_db(args.working, args.rosdep, args.rosdistro)
    pprint(rosdep_db)
    #update_repo(working_dir=args.working, repo_path=repo_path, repo_uri=stack_yaml['Release-Push'], first_release=args.first_release)

#     if not args.debian_inc:
#         tarball_name = '%(Package)s-%(Version)s.tar.gz' % stack_yaml
#         tarball_name = os.path.join(args.working, tarball_name)
#
#
#         print('Generating an upstream tarball --- %s' % tarball_name)
#         make_tarball(args.upstream,
#                      tarball_name)
#
#         import_orig(repo_path, tarball_name, stack_yaml['Version'])
    tags = []
    for debian_distro in args.distros:
        generate_deb(stack_yaml, ".", stamp, debian_distro, rosdep_db)
        commit_debian(stack_yaml, ".")
        #tags.append((gbp_sourcedebs(stack_yaml, repo_path, args.output)))
        tag_name = 'debian/ros_%(ROS_DISTRO)s_%(Version)s_%(Distribution)s' % stack_yaml
        print("tag: %s" % tag_name)
        call(".", ['git', 'tag', '-f', tag_name, '-m', 'Debian release %(Version)s' % stack_yaml])

    sys.exit(0)


if __name__ == "__main__":
    main(parse_options())
