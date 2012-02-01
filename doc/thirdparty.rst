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
