#!/bin/sh -e

/bin/echo "GOING AWAY"
exit 1

TOP=$(cd `dirname $0` ; /bin/pwd)
. $TOP/catkin_util.sh

<<<<<<< HEAD
usage () {
    cat <<EOF
Usage: $0 [args] <github_url>

args:
      -i
         the repo is new... don't be surprised that it contains no
         debian tags

      -v NEWVERSION
         new version is NEWVERSION

      -n
         Don't really do anything, just show commands (not working)

example:

      $0 -i -n 3.1.4 roscpp_core

EOF
    exit 1
}

while getopts "n:ih" opt; do
    case $opt in
        i)
            INITIALIZE_GBP=1
            /bin/echo "Expecting a fresh gbp repo... not checking for debian/ tags"
            ;;
        v)
            NEW_VERSION=$OPTARG
            /bin/echo "Will create new version ${boldon}$NEW_VERSION${boldoff}"
            ;;
        n)
            DO_NOTHING=1
            /bin/echo "Not actually runnign any commands."
            ;;
        *)
            usage
            ;;
    esac
done

GITHUB_URL=${!OPTIND}
to_github_uri GITHUB_URL
assert_is_remote_git_repo $GITHUB_URL

./catkin_patch_release.sh $GITHUB_URL $NEW_VERSION

##### HERE ######


=======
GITHUB_URL=$1
to_github_uri GITHUB_URL
assert_is_remote_git_repo $GITHUB_URL

if [ $# -gt 1 ] ; then
    NEW_VERSION=$2
    status "Will try to create new version $NEW_VERSION"
fi

./catkin_patch_release.sh $GITHUB_URL $NEW_VERSION
>>>>>>> c1393fe9d44e3e2f5a1cc1bcdb48fd6b346e75cb
TMPDIR=$(cat $TOP/tmp.dir)
cd $TMPDIR/gbp
$TOP/update_debian.py . fuerte
/bin/echo "${boldon}Debian files updated in gbp master branch.${boldoff}"
prompt_continue "Inspect now or just hit enter, you will get the chance to review the diffs before anything is pushed."

NEWTAGS=$(git for-each-ref --sort='-*authordate' --format='%(tag)' refs/tags/debian/\* | head -3)
/bin/echo "New tags are ${cyanf}$NEWTAGS${reset}"

for t in $NEWTAGS
do
    git checkout $t
    git clean -dfx
    git-buildpackage -uc -us -S --git-ignore-branch
done

/bin/echo "${boldon}If this all worked, you should be able to push the gbp repo.${reset}"
/bin/echo "I'll show you some logs/diffs."
prompt_continue "First, the git log of upstream.  This should contain only the stack version bump."
cd $TMPDIR/gbp
git checkout catkin
UPSTREAM_REPO=$(git config -f catkin.conf catkin.upstream)
UPSTREAM_TYPE=$(git config -f catkin.conf catkin.upstreamtype)
git checkout master

cd $TMPDIR/upstream
case $UPSTREAM_TYPE in
    git)
        git log --color -p -n1
        ;;
    svn)
        svn diff
        ;;
    hg)
        /bin/echo "finish me"
        exit 1
        ;;
esac

case $UPSTREAM_TYPE in
    git)
        THETAG=$(git for-each-ref --sort='-*authordate' refs/tags --count 1 --format='%(tag)')
        prompt_continue "The latest tag on upstream is ${boldon}$THETAG${boldoff}. I'll show you."
        git show $THETAG
        ;;
    hg)
        /bin/echo "finish me"
        exit 1
        ;;
    svn)
        /bin/echo "Latest upstream tag list:"
        svn ls $(dirname $UPSTREAM_REPO)/tags
        ;;
esac

prompt_continue "Now the git log of the gbp repo"
cd $TMPDIR/gbp
git log --color -p || /bin/true # the pager only returns 0 if you go the bottom of the file.

prompt_continue "Okay to push both?"

status "Pushing gbp"
cd $TMPDIR/gbp
git push --tags

cd $TMPDIR/upstream
case $UPSTREAM_TYPE in
    git)
        status "Pushing upstream"
        git push --tags
        ;;
    hg)
        /bin/echo "finish me"
        exit 1
        ;;
    svn)
        status "svn commit time... do it yourself."
        ;;
esac

