#!/bin/sh -ex

repos="catkin roscpp_core genpybindings std_msgs genpy gencpp genmsg"
export=`pwd`/export

for r in $repos
do
    if [ -d $r ] ; then
        pushd $r
        git fetch --all
        git reset --hard HEAD
        git clean -dfx
        popd
    else
        git clone git://github.com/wg-debs/$r.git
    fi
    pushd $r
    tags=$(git tag | grep '^debian/')
    for t in $tags
    do
        echo $t
        git checkout $t
        git buildpackage --git-ignore-new -S -uc -us
    done
    popd
done

