"Native" catkin projects
------------------------

Upstream
========

Fix up your project and commit a release.  *Upstream* is the repository where
your main development happens, and from which you would like to make packages.
An upstream release means to mark your code with a version number.

1. Ensure that :ref:`stack.yaml`\ 's ``Version`` field is correct, and
   meaningful: ``MAJOR.MINOR.PATCH``.  Specify all three numbers,
   including trailing zeros.

2. Create a tag whose name exactly corresponds to your Version number,
   and is pointing to the commit you would like to release.  Say my
   version is 0.1.1, in git i would do something like::

    git commit -m "Awesome 0.1.1 release"
    git tag -a $(catkin-version) -m "Releasing $(catkin-version)"
    git push --tags

   In hg::

     hg something or other

   In svn::

     svn something or other

Environment
===========

Make sure you have catkin in your ``PATH``: source an :ref:`envfiles`
as appropriate.


First Time Release
==================

Release Repo
++++++++++++

We're using git to manage releases. Think of the git repo as a release staging area.

Create a release repo::

  mkdir ecto-release
  cd ecto-release
  git init .

We need to tell the catkin tools how to get our upstream soureces.  This is done
using an orphan branch called catkin. The convenience command for this is::

  git catkin-set-upstream git://github.com/plasmodic/ecto.git git

You should now be on an orphaned branch in your repo called catkin.  If you look at
catkin.conf, you will see the upstream url and VCS type.

Let's get back to master branch if we're happy. This isn't strictly necessary,
but it helps me think.  master is the main branch from which debian's are created::

  git checkout master

Now that we have a catkin release repo, we can import from upstream::

  git catkin-import-upstream

This will clone the upstream, do a vcs export, and import it in the gbp repo.
It will look for a tag in in your upstream repo that corresponds to the Version if the
stack.yaml.

Now comes the debian generation...  This step creates/updates a debian folder and
creates tags that can easily be parsed and used for creating source or binary debians
from your release repo::

  git catkin-generate-debian fuerte

To test the debians try checking out a tag and using git-buildpackage to create
a binary debian::

  git checkout debian/ros_fuere_0.1.2_oneiric
  git clean -dxf
  git buildpackage -uc -us --git-ignore-new --git-ignore-branch

Push it Public
++++++++++++++

At this point you should add a remote and push::

  git add remote origin git@github.com:ethanrublee/ecto-release.git
  git push --all
  git push --tags


Subsequent Releases
===================

Choose a temporary directory somewhere in a quiet place, free from
distractions, where you can think.  We'll use ``STACK`` to represent
the stack being released.

* First verify that catkin's git utilities are in your path::

    % which git-catkin
    /home/catkin/test/src/catkin/bin/git-catkin

  If they aren't, source your :ref:`envfiles` as appropriate.

* Clone your :term:`gbp repository` (use a pushable uri for
  convenience)::

    % git clone git@github.com:wg-debs/STACK.git
    % cd STACK

  You should see tags for upstream source and debian releases::

    % git tag
    upstream/0.1.18
    upstream/0.1.19
    ...
    debian/ros_fuerte_0.2.2_lucid
    debian/ros_fuerte_0.2.2_natty
    debian/ros_fuerte_0.2.2_oneiric

  There may be a great many of these.  You'll see that there are three upstream branches::

    % git branch -r
    origin/HEAD -> origin/master
    origin/catkin
    origin/master
    origin/upstream

  Since you are about to import upstream source, you can verify what will be imported::

    % git show origin/catkin:catkin.conf
    [catkin]
            upstream = git@github.com:willowgarage/catkin.git
            upstreamtype = git

  (this is essentially catting the file ``catkin.conf`` from the
  origin's `catkin` branch.

* Import a new version of upstream.
  The upstream source will be retrieved from source control and
  imported in to this gbp repository. You'll be prompted to verify the
  upstream version::

    % git catkin-import-upstream
    STACK has branch catkin.
    Branch upstream set up to track remote branch upstream from origin.
    + git checkout catkin
    Switched to branch 'catkin'
    upstream repo: git@github.com:willowgarage/STACK.git
    upstream type: git
    Verifying a couple of things about the upstream git repo
    Verifying that git@github.com:willowgarage/STACK.git is a git repo...
    Yup, with 1 heads.
    Verifying that git@github.com:willowgarage/STACK.git is not a git-buildpackage repo
    Yup, no upstream branches.
    Cloning into ...

      ...

    What is the upstream version? [0.2.4]

      ...

* Now generate new debian tags::

    % git catkin-generate-debian fuerte
    catkin has branch catkin.
    catkin has branch upstream.
    M	debian/changelog
    Already on 'master'
    Your branch is ahead of 'origin/master' by 2 commits.
    The latest upstream tag in the release repo is upstream/0.2.4
    Upstream version is: 0.2.4
    + cd .tmp/25332/ && git clone git://github.com/ros/rosdep_rules.git
    Cloning into rosdep_rules...
    remote: Counting objects: 106, done.
    remote: Compressing objects: 100% (49/49), done.
    remote: Total 106 (delta 18), reused 94 (delta 7)
    Receiving objects: 100% (106/106), 11.05 KiB, done.
    Resolving deltas: 100% (18/18), done.

    ...

    [master d3cc805] + Creating debian mods for distro: oneiric, rosdistro: fuerte, upstream version: 0.2.4
     1 files changed, 1 insertions(+), 1 deletions(-)
    tag: debian/ros_fuerte_0.2.4_oneiric
    + cd . && git tag -f debian/ros_fuerte_0.2.4_oneiric -m Debian release 0.2.4
    Updated tag 'debian/ros_fuerte_0.2.4_oneiric' (was 0000000)


* Test it locally.  First checkout a tag that you would like to build::

    git checkout debian/ros_fuerte_0.1.2_oneiric

* Clean your checkout... there may be temporary files or directories
  laying around from previous steps. **This will delete all
  uncommitted content from your local repo**::

    % git clean -dxf
    Removing .tmp/

* Use git-buildpackage to build a binary debian.
  This command will generate a lot of output.  You may see a lot of
  errors about `dir-or-file-in-opt`, which is okay::

    % git buildpackage -uc -us --git-ignore-new --git-ignore-branch
    dh  clean
       dh_testdir
       dh_auto_clean
    	python2.6 setup.py clean -a
    running clean
    'build/lib.linux-x86_64-2.6' does not exist -- can't clean it
    ...
    E: ros-fuerte-STACK: dir-or-file-in-opt opt/ros/fuerte/share/STACK/
    ...
    Finished running lintian.

* A deb should have been produced in the parent directory.  Try
  installing it (needs sudo)::

    % ls ../*.deb
    ../ros-fuerte-STACK_0.2.4-0oneiric_amd64.deb
    dpkg -i ../ros-fuerte-STACK_0.2.4-0oneiric_amd64.deb

* If you're satisfied, push::

    % git remote -v
    origin	git@github.com:wg-debs/STACK.git (fetch)
    origin	git@github.com:wg-debs/STACK.git (push)
    % git push --all
    Total 0 (delta 0), reused 0 (delta 0)
    To git@github.com:wg-debs/STACK.git
    9793abc..987ceab  master -> master
    123d5d9..340fc7c  upstream -> upstream
    % git push --tags
    Counting objects: 4, done.
    Delta compression using up to 8 threads.
    Compressing objects: 100% (4/4), done.
    Writing objects: 100% (4/4), 664 bytes, done.
    Total 4 (delta 0), reused 0 (delta 0)
    To git@github.com:wg-debs/STACK.git
     * [new tag]         debian/ros_fuerte_0.2.4_lucid -> debian/ros_fuerte_0.2.4_lucid
     * [new tag]         debian/ros_fuerte_0.2.4_natty -> debian/ros_fuerte_0.2.4_natty
     * [new tag]         debian/ros_fuerte_0.2.4_oneiric -> debian/ros_fuerte_0.2.4_oneiric
     * [new tag]         upstream/0.2.4 -> upstream/0.2.4


tips and tricks
===============

This will create a rosinstall file for all repos in a github org::

  github_org_to_install()
  {
    for x in $(github orgs/$1/repos ssh_url+)
    do
    y=$(basename $x)
    echo "- git:
      uri: '$x'
      local-name: release-${y%.git}
      version: master
    "
    done
  }

Call like::

  github_org_to_install wg-debs
  
Version tools, for upstream releases::
    
    bump_minor()
    {
       git pull
       which=minor
       old_version=$(catkin-version)
       echo "old version: $old_version"
       catkin-bump-version $which
       version=$(catkin-version)
       echo "new version: $version"
       git commit stack.yaml -m "Bumping $which version $old_version ~> $version"
       git tag -a $version -m "$which release, $version"
       git push
       git push --tags
    }
    
