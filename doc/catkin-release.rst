Creating debian packages for "Native" catkin projects
-----------------------------------------------------

Environment
===========

First verify that catkin's ``git`` utilities are in your path::

    % which git-catkin
    /home/catkin/test/src/catkin/bin/git-catkin

If they aren't, source your :ref:`environment setup files <envfiles>` as appropriate.

Create your upstream release
============================

Fix up your project and commit a release.  *Upstream* is the
repository where your main development happens, and from which you
would like to make packages.  An upstream release means *to mark your
code with a version number*.

These instructions will walk you through creating a sample release.
We'll use ``STACK`` to represent the stack being released.

1. Ensure that :ref:`stack.yaml`\ 's ``Version`` field is correct, and
   meaningful: ``MAJOR.MINOR.PATCH``.  Specify all three numbers,
   including trailing zeros.  Remember to commit your change.

2. Create a tag whose name exactly corresponds to your ``Version`` number,
   and is pointing to the commit you would like to release.  If your
   version number is 0.1.1, depending on your version-control tool you
   would do something like:

   ``git``::

    git commit -m "0.1.1, minor release" stack.yaml
    git tag -a 0.1.1 -m "Releasing 0.1.1"
    git push
    git push --tags

   ``hg``::

     hg tag 0.1.1
     hg push

   ``svn`` (``https://path/to/STACK/branch`` is where the code is being developed, e.g. ``trunk``)::

     svn cp -m "Releasing 0.1.1" https://path/to/STACK/branch https://path/to/tags/STACK-0.1.1

First- time release
===================

Create the GBP repository
+++++++++++++++++++++++++

We're going to create a :term:`git-buildpackage (GBP) repository <GBP
repository>` using ``git`` and ``git-buildpackage``.
``git-buildpackage`` is a popular tool for creating and managing
debian packages using git repositories.  Think of the GBP repository
as a *release staging area*.

Create a new GBP repository for your release
(*reminder: "STACK" should be replaced with your stack's name*)::

  mkdir STACK-release
  cd STACK-release
  git init .

We need to tell the catkin tools how to get our upstream code using
the ``catkin-set-upstream`` command.  The command is called with the
upstream URL and VCS type (e.g. ``git``, ``svn``, ``hg``):

  ``git``::

    git catkin-set-upstream git://github.com/someproject/STACK.git git

  ``svn`` (set to the release tag URL)::

    git catkin-set-upstream https://path/to/STACK/tag/STACK-0.1.1 svn

You should now be on an orphaned branch in your repository, called
``catkin``, which was created by the ``catkin-set-upstream`` command.
If you look at ``catkin.conf``, you will see the upstream URL and VCS
type.

Now, let's get back to ``master`` branch if everything looks
good::

  git checkout master

This isn't strictly necessary, but it helps show how things are
configured. ``master`` is the main branch from which debian packages
are created.

Now that we have a catkin release repository, we can import from
upstream using the ``catkin-import-upstream`` command::

  git catkin-import-upstream

This will export the upstream repository and import it in GBP
repository.  The exact behavior depends on your version control tool:

- ``git``: catkin will look for a tag in in your upstream repo that
  corresponds to the ``Version`` in the ``stack.yaml``.

- ``svn``: catkin will export the upstream URL that you provided.
  Please make sure this is the exact code corresponding to
  ``Version``.

Creating the debian package
+++++++++++++++++++++++++++

We will now use the ``catkin-generate-debian`` command to create our
debian package configuration files.  This command creates/updates a
``debian/`` folder.  It also creates git tags that are used for
creating source and binary debians from your GBP repository.  We're
going to run this command with the ``fuerte`` argument to setup a
release for the ``fuerte`` distribution::

  git catkin-generate-debian fuerte

To test the debians try checking out a tag and using ``git
buildpackage`` to create a binary debian.  In our example, we released
version ``0.1.1`` and created a debian package for ``fuerte``.  This
means the tag for the Ubuntu Oneiric platform is
``debian/ros-fuerte-STACK-0.1.1_oneiric`` tag.  To build a debian package
for this platform::

  git checkout debian/ros-fuerte-STACK-0.1.1_oneiric
  git clean -dxf
  git buildpackage -uc -us --git-ignore-branch --git-ignore-new   # on lucid, omit --git-ignore-new

Push it public
++++++++++++++

Now it's time to save your work and make it public.  Use git to add a
``remote`` repository for your GBP repository.  This remote repository
should be public (e.g. on GitHub).  ``push`` your data to the remote
repository to make it public. Remember to substitute the correct
URL/username for your project::

  git remote add origin git@github.com:project/STACK-release.git
  git push --all
  git push --tags


Subsequent Releases
===================

Choose a temporary directory somewhere in a quiet place, free from
distractions.

Clone your GBP repository
+++++++++++++++++++++++++

Clone your :term:`GBP repository` (use a pushable URI for convenience)::

  git clone git@github.com:wg-debs/STACK.git
  cd STACK

.. note:: **Optional**

  After you clone, you may want to inspect your repository to get familiar with how things work and to check that everything looks good. You should see tags for upstream source and debian releases::
  
    % git tag
    upstream/0.1.18
    upstream/0.1.19
    ...
    debian/ros-fuerte-STACK-0.2.2_lucid
    debian/ros-fuerte-STACK-0.2.2_oneiric
  
  There may be a great many of these.  You'll see that there are three
  upstream branches::
  
    % git branch -r
    origin/HEAD -> origin/master
    origin/catkin
    origin/master
    origin/upstream
  
  Since you are about to import upstream source, you can verify what
  will be imported::
  
    % git show origin/catkin:catkin.conf
    [catkin]
            upstream = git@github.com:project/STACK.git
            upstreamtype = git
  
  This is essentially catting the file ``catkin.conf`` from the
  origin's ``catkin`` branch.
  

SVN: update your upstream URL
+++++++++++++++++++++++++++++

For ``svn`` it is important to update this to point to the new release tag::

   git catkin-set-upstream https://path/to/STACK/tags/STACK-0.1.1 svn

Import a new version of upstream
++++++++++++++++++++++++++++++++

Now you need to resync with your upstream source using
``catkin-import-upstream``.  The upstream source will be retrieved
from source control and imported in to this :term:`GBP
repository`. You'll be prompted to verify the upstream version::

  git catkin-import-upstream


..

  Example output::

    % git catkin-import-upstream
    STACK has branch catkin.
    Branch upstream set up to track remote branch upstream from origin.
    + git checkout catkin
    Switched to branch 'catkin'
    upstream repo: git@github.com:project/STACK.git
    upstream type: git
    Verifying a couple of things about the upstream git repo
    Verifying that git@github.com:project/STACK.git is a git repo...
    Yup, with 1 heads.
    Verifying that git@github.com:project/STACK.git is not a git-buildpackage repo
    Yup, no upstream branches.
    Cloning into ...

      ...

    What is the upstream version? [0.2.4]

      ...

Create the debian packaging
+++++++++++++++++++++++++++

Now we need to generate git tags for our release using the ``catkin-generate-debian`` command, which is called with the name of the ROS distribution codename. In this example, we are going to release to the ``fuerte`` ROS distribution::

  git catkin-generate-debian fuerte

..

  Example output::

    % git catkin-generate-debian fuerte
    catkin has branch catkin.
    catkin has branch upstream.
    M    debian/changelog
    Already on 'master'
    Your branch is ahead of 'origin/master' by 2 commits.
    The latest upstream tag in the release repo is upstream/0.1.1
    Upstream version is: 0.1.1
    + cd .tmp/25332/ && git clone git://github.com/ros/rosdep_rules.git
    Cloning into rosdep_rules...
    remote: Counting objects: 106, done.
    remote: Compressing objects: 100% (49/49), done.
    remote: Total 106 (delta 18), reused 94 (delta 7)
    Receiving objects: 100% (106/106), 11.05 KiB, done.
    Resolving deltas: 100% (18/18), done.

    ...

    [master d3cc805] + Creating debian mods for distro: oneiric, rosdistro: fuerte, upstream version: 0.1.1
     1 files changed, 1 insertions(+), 1 deletions(-)
    tag: debian/ros-fuerte-STACK-0.1.1_oneiric
    + cd . && git tag -f debian/ros-fuerte-STACK-0.1.1_oneiric -m Debian release 0.1.1
    Updated tag 'debian/ros-fuerte-STACK-0.1.1_oneiric' (was 0000000)


Now we need to verify that your tag got created locally.  Git tag will show them::

  % git tag
  debian/ros-fuerte-STACK-1.7.3_lucid
  debian/ros-fuerte-STACK-1.7.3_oneiric
  debian/ros-fuerte-STACK-1.7.4_lucid
  debian/ros-fuerte-STACK-1.7.4_oneiric
  upstream/1.7.3
  upstream/1.7.4

.. note:: **Optional**

  You may test the package building process if you have the package's
  system dependencies installed.  ``checkout`` one of the `new` tags
  corresponding to the version you are releasing and matching the ubuntu
  version of your local host.  In this example, we checkout for Ubuntu
  Oneiric::

      git checkout debian/ros-fuerte-STACK-1.7.4_oneiric

  It may complain about ``detached HEAD``, this would be a good time to
  `Check Your Head
  <http://upload.wikimedia.org/wikipedia/en/d/d1/Beastieboys_checkyourhead.jpg>`_.
  Next, ``clean`` your checkout. **This will delete all uncommitted
  content from your local repo**. There may be temporary files or
  directories laying around from previous steps that have to be
  removed. ::

    % git clean -dxf
    Removing .tmp/

  Use ``git buildpackage`` to build a binary debian. This command will
  generate a lot of output.  You may see a lot of errors about
  "dir-or-file-in-opt", which is okay::

    git buildpackage -uc -us --git-ignore-new  # on lucid omit --git-ignore-new

  But this may fail if you haven't installed the system dependencies locally::

    dpkg-buildpackage: host architecture amd64
    dpkg-checkbuilddeps: Unmet build dependencies: libboost1.46-all-dev ros-fuerte-rospack ros-fuerte-catkin ros-fuerte-rospkg
    dpkg-buildpackage: warning: Build dependencies/conflicts unsatisfied; aborting.
    dpkg-buildpackage: warning: (Use -d flag to override.)
    debuild: fatal error at line 1340:
    dpkg-buildpackage -rfakeroot -D -us -uc -i -I failed
    debuild -i -I returned 29
    Couldn't run 'debuild -i -I -uc -us'

  Which isn't a complete catastrophe.  Cheer up.  If it succeeded, a deb
  should have been produced in the parent directory.  Try installing it
  (requires ``sudo`` permission)::

      % ls ../*.deb
      ../ros-fuerte-STACK_0.1.1-0oneiric_amd64.deb
      % dpkg -i ../ros-fuerte-STACK_0.1.1-0oneiric_amd64.deb

If this worked and you're satisfied, or if you are just feeling lucky,
``push`` your packaging to the public::

  git push --all
  git push --tags

.. 

  Example output::
  
    % git remote -v
    origin	git@github.com:project/STACK-release.git (fetch)
    origin	git@github.com:project/STACK-release.git (push)
    % git push --all
    Total 0 (delta 0), reused 0 (delta 0)
    To git@github.com:project/STACK-release.git
    9793abc..987ceab  master -> master
    123d5d9..340fc7c  upstream -> upstream
    % git push --tags
    Counting objects: 3, done.
    Delta compression using up to 6 threads.
    Compressing objects: 100% (3/3), done.
    Writing objects: 100% (3/3), 498 bytes, done.
    Total 3 (delta 0), reused 0 (delta 0)
    To git@github.com:project/STACK-release.git
     * [new tag]         debian/ros-fuerte-STACK-0.1.1_lucid -> debian/ros-fuerte-STACK-0.1.1_lucid
     * [new tag]         debian/ros-fuerte-STACK-0.1.1_oneiric -> debian/ros-fuerte-STACK-0.1.1_oneiric
     * [new tag]         upstream/0.1.1 -> upstream/0.1.1
   

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

Build of workspace::

    mkdir /tmp/ws
    cd /tmp/ws
    wget https://raw.github.com/willowgarage/catkin/master/test/release_repos.yaml
    git catkin install release_repos.yaml
    catkin-build-debs-of-workspace

    
