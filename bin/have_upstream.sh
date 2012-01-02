#!/bin/bash

TOP=$(cd `dirname $0` ; /bin/pwd)
. $TOP/catkin_util.sh

github_api REPONAMES orgs/wg-debs/repos name+

/bin/echo "urls=$REPOS"

for reponame in $REPONAMES
do
    github_api SHA repos/wg-debs/$reponame/git/refs/heads/catkin object sha
    /bin/echo "$reponame $SHA"
done


