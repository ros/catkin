3rd party releases
------------------

3rd party releases required the finest grain control over the process, and are good to examine
in order to understand the catkin packaging mechanisms.

Make sure you have catkin in your bin. To enter an interactive shell with the right environment set::

  /path/to/catkin/install/or/build/env.sh

3rd party release example
^^^^^^^^^^^^^^^^^^^^^^^^^

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

    Depends: cmake,
        libtbb-dev,
        libgtest-dev,
        python,
        python-numpy,
        boost

    Description: FLANN is a library for performing fast approximate nearest
        neighbor searches in high dimensional spaces. It contains a collection
        of algorithms we found to work best for nearest neighbor search and
        a system for automatically choosing the best algorithm and
        optimum parameters depending on the dataset.

    Author: Marius Muja <mariusm@cs.ubc.ca>
    Maintainer: Ethan Rublee <erublee@willowgarage.com>
    Homepage: http://people.cs.ubc.ca/~mariusm/index.php/FLANN/FLANN

    Catkin-CopyrightType:  willowgarage
    Catkin-DebRulesType: cmake
    EOF
    git add catkin.conf stack.yaml
    git commit -m "Adding catkin.conf and stack.yaml"

If you want to pick a specific package name add a "Package" key to stack.yaml with the 
debian package name you want as the output. The default value is "ros-%(rosdistro)s-%(Catkin-PackageName)s
Also supported is INSTALL_PREFIX as a key, to set the install prefix.  Default: /opt/ros/%(rosdistro)s/

For Precise and above, you need to do the following too (from
http://raphaelhertzog.com/2010/11/18/4-tips-to-maintain-a-3-0-quilt-debian-source-package-in-a-vcs/ or 
2011-09-23 http://wiki.debian.org/qa.debian.org/FTBFS):

::

    $ echo "single-debian-patch" >> debian/source/local-options
    $ cat >debian/source/patch-header <<END
    This patch contains all the Debian-specific
    changes mixed together. To review them
    separately, please inspect the VCS history
    at http://git.debian.org/?=collab-maint/foo.git

    END


Merge the catkin branch::

    git checkout master
    git merge -X theirs catkin

Verify that it actually overlayed catkin onto the master branch.

Now generate debian files::

    catkin-generate-debian fuerte

Test it locally::

    git clean -dxf
    git checkout debian/ros_fuerte_1.7.1_oneiric
    git buildpackage -uc -us --git-ignore-branch --git-ignore-new  # on lucid, omit --git-ignore-new

Now push all branches and tags to a remote::

    git remote add origin git@github.com:ros/flann.git
    git push --all
    git push --tags

If you are working with an existing repo, either run ``gbp-clone`` or make sure to fetch all branches.

Subsequent Releases
^^^^^^^^^^^^^^^^^^^

Choose a temporary directory somewhere in a quiet place, free from
distractions.

Clone your GBP repository
+++++++++++++++++++++++++

Clone your :term:`GBP repository` (use a pushable URI for convenience)::

  git clone git@github.com:wg-debs/STACK-release.git
  cd STACK-release

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
  

Create a tarball of the new updated code
++++++++++++++++++++++++++++++++++++++++

For ``svn`` use ``svn export`` to remove the ``.svn`` folders.::

  tar -cf foo.tgz foo

Put that tarball somewhere (not in the git folder).

Import a new version of upstream
++++++++++++++++++++++++++++++++

You need to import the tarball::

  git checkout master
  git import-orig _path_to_your_tarball

For some reason, I have to do ``git checkout master`` in the first place (to initialize something in git ...).

..

  Example output::

    What is the upstream version? [] 2.3.9
    gbp:info: Importing '/home/vrabaud/opencv.tgz' to branch 'upstream'...
    gbp:info: Source package is ros-fuerte-opencv2
    gbp:info: Upstream version is 2.3.9
    gbp:info: Merging to 'master'
    Removing 3rdparty/CMakeLists.txt
    Removing 3rdparty/ffmpeg/CMakeLists.txt
    Removing 3rdparty/libtiff/tif_apple.c
    Removing 3rdparty/libtiff/tif_config.h
    Removing 3rdparty/libtiff/tiffconf.h
    Removing 3rdparty/zlib/.cvsignore
    Auto-merging 3rdparty/zlib/zconf.h.cmakein
    Removing android/CMakeCache.android.initial.cmake
    Auto-merging apps/haartraining/CMakeLists.txt
    Removing cmake/templates/opencv.pc.cmake.in
    Auto-merging doc/tutorials/core/mat_the_basic_image_container/mat_the_basic_image_container.rst
    Removing modules/traincascade/CMakeLists.txt
    Auto-merging samples/cpp/openni_capture.cpp
    Removing samples/gpu/optical_flow.cpp
    Merge made by recursive.
    gbp:info: Successfully imported version 2.3.9 of /home/vrabaud/opencv.tgz

Update the stack.yaml
+++++++++++++++++++++

Switch to the catkin branch and modify whatever you want in there (at least the stack.yaml, but patches too maybe)::

  git checkout catkin

Patches
+++++++

If you have patches to commit, simply put the new file with the corresponding hierarchy in the catkin branch.
No need to deal with the ``debian/patches`` folder.

Create the debian packaging
+++++++++++++++++++++++++++

Now you can relax and repeat the instructions from above.::

    git checkout master
    git merge -X theirs catkin

Verify that it actually overlayed catkin onto the master branch.

Now generate debian files::

    catkin-generate-debian fuerte

Test it locally::

    git clean -dxf
    git checkout debian/ros_fuerte_1.7.1_oneiric
    git buildpackage -uc -us --git-ignore-branch --git-ignore-new  # on lucid, omit --git-ignore-new

If that worked, push all branches and tags to the already existing remote::

    git push --all
    git push --tags
