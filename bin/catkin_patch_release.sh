#!/bin/bash -ex

TOP=$(cd `dirname $0` ; /bin/pwd)
. $TOP/catkin_util.sh

usage () {
    cat <<EOF
Usage: $0 [args] <gbp_repo>

args:
      -g gbp_repo
         (writable) uri of gbp git repo
         this can be just the name of the stack, it will get transformed
         to git@github.com:wg-debs/[gbp_repo].git per git setting
         'catkin.gbproot'

      -i
         the repo is new... don't be surprised that it contains no
         debian tags

      -v NEWVERSION
         new version is NEWVERSION

      -n
         Don't really do anything, just show commands (not working)

      -c
         Do *not* commit/push tags to upstream

      -m
         Merge the catkin branch ontop of master.
         Prefer the stack.yaml in the master branch, don't look
         for it in the upstream. This is useful for 3rd party
         package maintenance.

example:

      $0 -i git@github.com:wg-debs/roscpp_core.git

EOF
    exit 1
}

EXPECT_GBP_REPO_IS_UNINITIALIZED=0
TAG_UPSTREAM=1
READ_UPSTREAM_STACK=1
MERGE_CATKIN_BRANCH=0
while getopts "cinmv:g:" opt; do
    case $opt in
        i)
            EXPECT_GBP_REPO_IS_UNINITIALIZED=1
            /bin/echo "Expecting a fresh gbp repo... not checking for debian/ tags"
            ;;
        g)
            GBP_REPO=$OPTARG
            /bin/echo "GBP repo via ${boldon}$GBP_REPO${boldoff}"
            ;;
        v)
            NEW_VERSION=$OPTARG
            /bin/echo "Will create new version ${boldon}$NEW_VERSION${boldoff}"
            ;;
        n)
            DO_NOTHING=1
            SUBSHELL="/bin/echo '>>>' "
            /bin/echo "Not actually running commands."
            ;;
        c)
            TAG_UPSTREAM=0
            /bin/echo "I won't push/commit any tags to upstream."
            ;;
        m)
            READ_UPSTREAM_STACK=0
            MERGE_CATKIN_BRANCH=1
            /bin/echo "I will ignore the stack.yaml in upstream."
            /bin/echo "I will merge the catkin orphan into the master branch before packaging."
            ;;
        *)
            usage
            ;;
    esac
done

if [ ${OPTIND} -le $# ] ; then
    usage
fi

to_github_uri GBP_REPO
if [ $EXPECT_GBP_REPO_IS_UNINITIALIZED -eq 0 ] ; then
    assert_is_gbp_repo $GBP_REPO
fi
GBP_CLONE=$TMPDIR/gbp
git clone -b catkin $GBP_REPO $GBP_CLONE
assert_nonempty $GBP_CLONE

if [ $EXPECT_GBP_REPO_IS_UNINITIALIZED -eq 0 ] ; then
    if [ -z "$NEW_VERSION" ] ; then
        get_latest_gbp_version $GBP_CLONE
    fi
fi
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
if [ $READ_UPSTREAM_STACK -ne 0 ] ; then
    read_stack_yaml $UPSTREAM_CLONE/stack.yaml
    /bin/echo "Upstream's stack.yaml has version ${boldon}$VERSION_FULL${reset}"
else
    read_stack_yaml $GBP_CLONE/stack.yaml
    cp $GBP_CLONE/stack.yaml $UPSTREAM_CLONE/stack.yaml
    /bin/echo "GBP stack.yaml has version ${boldon}$VERSION_FULL${reset}"
fi

if [ $EXPECT_GBP_REPO_IS_UNINITIALIZED -ne 1 ] ; then
    get_latest_gbp_version $GBP_CLONE
else
    /bin/echo "Not looking for latest gbp version since this is a brand new repo."
fi

if [ "$GBP_MAJOR.$GBP_MINOR.$GBP_PATCH" = "$VERSION_FULL" ] ; then
    /bin/echo "${boldon}stack.yaml's version ${redf}($VERSION_FULL)${whitef} matches latest gbp tag${reset}"
    NEW_PATCH=$(expr $GBP_PATCH + 1)
    NEW_VERSION=$GBP_MAJOR.$GBP_MINOR.$NEW_PATCH
    /bin/echo "Would you like to create a new relase with version ${redf}${boldon}$NEW_VERSION${reset} [y/N]?"
    read ANS
    if [ $ANS != 'y' ] ; then
        bailout "aborting at user's request"
    fi
else
    status "The new version will be ${boldon}$VERSION_FULL${reset}, read from the stack.yaml."
    status "Is this ok? [y/N]"
    read ANS
    if [ $ANS != 'y' ] ; then
        bailout "aborting at user's request"
    fi
    NEW_VERSION=$VERSION_FULL
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
        [ $TAG_UPSTREAM -eq 1 ] && $SUBSHELL git tag -m "$TAGMSG" $NEW_VERSION
        ;;
    hg)
        $SUBSHELL hg commit -m "$COMMITMSG"
        [ $TAG_UPSTREAM -eq 1 ] && $SUBSHELL hg tag -m "$TAGMSG" $NEW_VERSION
        ;;
    svn)
        $SUBSHELL svn commit -m "$COMMITMSG"
        TAGURL=$(dirname $UPSTREAM_REPO)/tags/$(basename $GBP_REPO .git)-$NEW_VERSION
        [ $TAG_UPSTREAM -eq 1 ] && $SUBSHELL svn cp $UPSTREAM_REPO $TAGURL -m "$TAGMSG"
        ;;
esac

status "The upstream's ${boldon}stack.yaml${boldoff} is updated and committed,"
status "and the upstream repo has been tagged."

status "Importing new upstream into gbp repo"

UPSTREAM_BASE=$(basename $UPSTREAM_REPO .git)

#
#  exporting for git-import-orig consumption
#
cd $UPSTREAM_CLONE
repo_export $UPSTREAM_TYPE $UPSTREAM_CLONE $TMPDIR/export $NEW_VERSION



cd $GBP_CLONE
if [ $EXPECT_GBP_REPO_IS_UNINITIALIZED -eq 1 ] ; then
    git checkout -b upstream
else
    git checkout -b upstream origin/upstream
fi
git checkout -b master origin/master

# older gbp needs things zipped
status "Will now git-import-orig...  it doesn't matter what you enter as the source package name."
git-import-orig -u $NEW_VERSION $TMPDIR/export.tar.gz

if [ $MERGE_CATKIN_BRANCH -ne 0 ] ; then
    status "Will now merge catkin branch into master."
    #this appears to not do anything ....
    #git merge --no-commit -X theirs catkin
    #git commit --allow-empty -m "Merging catkin directory."
    git archive --format=tar catkin > $TMPDIR/catkin.tar
    tar -xvf $TMPDIR/catkin.tar
    git add .
    git commit --allow-empty -m "Merging catkin directory."
fi

status "Upstream imported into new gbp repo and committed."
status "Now you should update the debian files."
status "The relevant repos are in $TMPDIR (also in file tmp.dir)"

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
prompt_continue "First, the log of upstream.  This should contain the stack.yaml version change and .hgtags if upstream is hg."
cd $TMPDIR/gbp
git checkout catkin
UPSTREAM_REPO=$(git config -f catkin.conf catkin.upstream)
UPSTREAM_TYPE=$(git config -f catkin.conf catkin.upstreamtype)
git checkout master

if [ $TAG_UPSTREAM -ne 0 ]; then
    cd $TMPDIR/upstream
    case $UPSTREAM_TYPE in
        git)
            git log --color -p -n1
            ;;
        svn)
            svn diff
            ;;
        hg)
            # one for the tag, one for the commit
            hg log -gp -l 2
            ;;
    esac

    case $UPSTREAM_TYPE in
        git)
            THETAG=$(git for-each-ref --sort='-*authordate' refs/tags --count 1 --format='%(tag)')
            prompt_continue "The latest tag on upstream is ${boldon}$THETAG${boldoff}. I'll show you."
            git show $THETAG
            ;;
        hg)
            /bin/echo "Current hg tags"
            hg tags
            ;;
        svn)
            /bin/echo "Latest upstream tag list:"
            svn ls $(dirname $UPSTREAM_REPO)/tags
            ;;
    esac
fi
prompt_continue "Now the git log of the gbp repo"
cd $TMPDIR/gbp
git log --color -p || /bin/true # the pager only returns 0 if you go the bottom of the file.

prompt_continue "Okay to push both?"

status "Pushing gbp"
cd $TMPDIR/gbp

git push --all
git push --tags

if [ $TAG_UPSTREAM -ne 0 ]; then
    cd $TMPDIR/upstream
    case $UPSTREAM_TYPE in
        git)
            status "Pushing upstream"
            if [ $TAG_UPSTREAM -eq 1 ] ; then
                git push --tags
            else
                git push
            fi

            ;;
        hg)
            hg push
            ;;
        svn)
            status "svn commit time... do it yourself."
            ;;
    esac
else
    status "You requested to not tag upstream so not pushing to upstream."
fi
