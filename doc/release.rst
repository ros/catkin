Release 
-------

Git will be abused by catkin to manage releases and packaging of
projects.  Packaging, in the current context refers to the eventual
creation of binary debians.

There are a few use cases of packaging that it should handle with varying amounts manual labor.

1. 3rdparty releases
2. native Catkin releases
3. Existing debian packages

For 1 and 2, the packages that will be created by catkin and the build farm will
be installed to a **FHS** that is determined by the ``CMAKE_INSTALL_PREFIX`` variable.  For
ROS, this is ``/opt/ros/X`` where ``X`` is a ros distrubution.  This is the assumed case if you are
making releases with catkin.  However, there should be no reason that debians can not be
manufactured that install to ``/``.

For 3, these packages are ingested using a dput of either source or binary debians and the build
system will not change the package in any way. This form of release is highly discouraged except
for libraries that are very stable.

tools
=====

``git catkin-gbp`` is a set of tools that manage catkin specific git-buildpackage, *gbp*, repos.
This is the packaging tool, and expects upstream to have created a release in some manner.

A release of upstream means that you have incremented your version number, e.g. in the stack.yaml,
and have created a tag, branch, tarball, or written down a revision on a napkin.

``git catkin-gbp`` will take the release, and either create or update a release repository. It is from
this repository that source debians will be generated, using git-buildpackage and a little trickery.

3rd party releases
==================

3rd party releases required the finest grain control over the process, and are good to examine
in order to understand the catkin packaging mechanisms.

A prereq for these notes is that you have a recent version of catkin, and the
catkin/bin folder is in your path::

    git clone git://github.com/willowgarage/catkin.git /tmp/catkin
    export PATH=/tmp/catkin/bin:$PATH


3rd party release example
+++++++++++++++++++++++++

Let's do flann. Setup a git repo::
    
    mkdir flann
    (cd flann && git init)

Grab a tarball from somewhere::

    wget https://github.com/wg-debs/flann/tarball/upstream/1.7.1 -O flann-1.7.1.tar.gz

Git import orig it::
    
    cd flann
    git import-orig ../flann-1.7.1.tar.gz

Create a cakin orphan branch::

    git checkout --orphan catkin
    git rm -rf .
    cat > catkin.conf <<EOF
    [catkin]
        upstream =  https://github.com/wg-debs/flann/tarball/upstream/1.7.1 
        upstreamtype = manual
    EOF
    cat > stack.yaml <<EOF
    Catkin-ProjectName: flann
    Version: 1.7.1

    Depends: libtbb-dev, libgtest-dev, python, python-numpy, libboost-all-dev, libboost-mpi-dev

    Description: FLANN is a library for performing fast approximate nearest neighbor searches in high dimensional spaces. It contains a collection of algorithms we found to work best for nearest neighbor search and a system for automatically choosing the best algorithm and optimum parameters depending on the dataset.

    Author: Marius Muja <mariusm@cs.ubc.ca>
    Maintainer: Ethan Rublee <erublee@willowgarage.com>
    Homepage: http://people.cs.ubc.ca/~mariusm/index.php/FLANN/FLANN

    Catkin-CopyrightType:  willowgarage
    Catkin-DebRulesType: cmake 
    EOF
    git add catkin.conf stack.yaml
    git commit -m "Adding catkin.conf and stack.yaml"

Merge the catkin branch::

    git checkout master
    git merge -X theirs catkin

Verify that it actually overlayed catkin onto the master branch. Now generate debian files::

    update_debian.py . fuerte

Test it locally::
    
    git clean -dxf
    git checkout debian/ros_fuerte_1.7.1_oneiric
    git buildpackage -S -uc -us --git-ignore-branch

Build that package with pbuilder or something...

Now push all branches and tags to a remote::

    ##add an origin if you don't have one.    
    #git add remote origin git@github.com:ros/flann.git
    git push --all
    git push --tags

If you are working with an existing repo, either run ``gbp-clone`` or make sure to fetch all branches.
