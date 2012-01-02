#!/bin/bash -e

TOP=$(cd `dirname $0` ; /bin/pwd)
. $TOP/catkin_util.sh

if [ $# -lt 1 ] ; then
    /bin/echo "Usage: $0 <gbp_repo>"
    /bin/echo "gbp_repo will be determined automagically (in wg-debs) if not specified"
    exit 1
fi

if [ $# -gt 1 ] ; then
    NEW_VERSION=$2
fi

#if [ "$1" = "-n" ] ; then
    SUBSHELL="/bin/echo '>>>' "
#    shift
#else
#    SUBSHELL=""
#fi

GBP_REPO=$1
to_github_uri GBP_REPO
assert_is_gbp_repo $GBP_REPO

GBP_CLONE=$TMPDIR/gbp
git clone -b catkin $GBP_REPO $GBP_CLONE
assert_nonempty $GBP_CLONE
get_latest_gbp_version $GBP_CLONE
cd $GBP_CLONE
if [ ! -f catkin.conf ] ; then
    bailout "Could not find UPSTREAM (containing upstream URL) on catkin branch of gbp repo"
fi
UPSTREAM_REPO=$(git config -f catkin.conf catkin.upstream)
UPSTREAM_TYPE=$(git config -f catkin.conf catkin.upstreamtype)

status "Upstream repo is $UPSTREAM_REPO, type $UPSTREAM_TYPE"

if [ $UPSTREAM_TYPE = "git" ] ; then
    status "Verifying a couple of things about the upstream git repo"
    assert_is_remote_git_repo $UPSTREAM_REPO
    assert_is_not_gbp_repo $UPSTREAM_REPO
fi

UPSTREAM_CLONE=$TMPDIR/upstream
repo_clone $UPSTREAM_TYPE $UPSTREAM_REPO $UPSTREAM_CLONE
read_stack_yaml $UPSTREAM_CLONE/stack.yaml
/bin/echo "Upstream's stack.yaml has version ${boldon}$VERSION_FULL${reset}"

if [ "$GBP_MAJOR.$GBP_MINOR.$GBP_PATCH" = "$VERSION_FULL" ] ; then
    /bin/echo "${boldon}stack.yaml's version ${redf}($VERSION_FULL)${whitef} matches latest gbp tag${reset}"
    NEW_PATCH=$(expr $GBP_PATCH + 1)
    NEW_VERSION=$GBP_MAJOR.$GBP_MINOR.$NEW_PATCH
    /bin/echo "Would you like to create a new relase with version ${redf}${boldon}$NEW_VERSION${reset} [y/N]?"
    read ANS
    if [ $ANS != 'y' ] ; then
        bailout "aborting at user's request"
    fi
fi

if [ -z "$NEW_VERSION" ] ; then
    bailout "What should the new version be?"
fi

cd $UPSTREAM_CLONE
perl -ni -e "s/^Version: .*\$/Version: $NEW_VERSION/ ; print" stack.yaml
cat stack.yaml
COMMITMSG="catkin script created new patch version $NEW_VERSION"
TAGMSG="catkin script created new patch version $NEW_VERSION"

case $UPSTREAM_TYPE in
    git)
        git add stack.yaml
        git commit -m "$COMMITMSG"
        git tag -m "$TAGMSG" $NEW_VERSION
        ;;
    hg)
        /bin/echo "finish me"
        exit 1
        ;;
    svn)
        $SUBSHELL svn commit -m "$COMMITMSG"
        TAGURL=$(dirname $UPSTREAM_REPO)/tags/$(basename $GBP_REPO .git)-$NEW_VERSION
        $SUBSHELL svn cp $UPSTREAM_REPO $TAGURL -m "$TAGMSG"
        ;;
esac

status "The upstream's ${boldon}stack.yaml${boldoff} is updated and committed,"
status "and the upstream repo has been tagged."

status "Importing new upstream into gbp repo"

UPSTREAM_BASE=$(basename $UPSTREAM_REPO .git)

#
#  exporting for git-import-orig consumption
#
set -x
cd $UPSTREAM_CLONE
case $UPSTREAM_TYPE in
    git)
        git archive -o ../upstream.tar $NEW_VERSION
        gzip ../upstream.tar
        ;;
    svn)
        tar --exclude-vcs -cvzf ../upstream.tar.gz .
        ;;
    hg)
        /bin/echo "finish me"
        exit 1
        ;;
esac


cd $GBP_CLONE
git checkout -b upstream origin/upstream
git checkout -b master origin/master

# older gbp needs things zipped
git-import-orig -u $NEW_VERSION ../upstream.tar.gz

# fixme: check that import worked as expected
/bin/echo $TMPDIR > $TOP/tmp.dir

status "Upstream imported into new gbp repo and committed."
status "Now you should update the debian files."
status "The relevant repos are in $TMPDIR (also in file tmp.dir)"