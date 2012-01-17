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

Make sure you have catkin in your bin. To enter an interactive shell with the right environment::

  /path/to/catkin/install/or/build/env.sh


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

First clone your release repo (use a pushable uri for convenience)::

  git clone git@github.com:ethanrublee/ecto-release.git
  cd ecto-release

Import a new version of upstream::

  git catkin-import-upstream

If you're happy with it, generate new debian tags::

  git catkin-generate-debian fuerte

Test it locally.  First checkout a tag that you would like to build::

  git checkout debian/ros_fuerte_0.1.2_oneiric

Clean your repo **WARNING This will delete all uncommitted content from your local repo**::

  git clean -dxf

Use git-buildpackage to build a binary debian::

  git buildpackage -uc -us --git-ignore-new --git-ignore-branch

Try installing it (needs sudo)::

  dpkg -i ../ros-fuerte-ecto*.deb

If you're satisfied, push it::

  git push --all
  git push --tags


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
