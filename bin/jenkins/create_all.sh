for repo_uri in git://github.com/wg-debs/roscpp_core.git \
    git://github.com/wg-debs/genpybindings.git \
    git://github.com/wg-debs/std_msgs.git \
    git://github.com/wg-debs/genpy.git \
    git://github.com/wg-debs/gencpp.git \
    git://github.com/wg-debs/genmsg.git \
    git://github.com/wg-debs/catkin.git
do
    ./create_debjobs.py fuerte $repo_uri --dscs export --commit
done
