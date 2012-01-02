#!/bin/sh -ex

TOP=$(cd `dirname $0` ; /bin/pwd)
. $TOP/catkin_util.sh

GITHUB_URL=$1
to_github_uri GITHUB_URL
assert_is_remote_git_repo $GITHUB_URL

if [ $# -gt 1 ] ; then
    NEW_VERSION=$2
    status "Will try to create new version $NEW_VERSION"
fi

./catkin_patch_release.sh $GITHUB_URL $NEW_VERSION
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

