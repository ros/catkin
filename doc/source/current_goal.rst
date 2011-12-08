Current Goal
------------

to the right (towards the user)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* :term:`dry` build of ``ros_tutorials``, against a :term:`wet` built ``/opt/ros/fuerte``
* successful ``roslaunch`` and ``rosrun`` of tutorials
* then install ros_tutorials (dry: to PREFIX/stacks); rosruns/launches still work
* rm the ros tutorials from PREFIX
* a :term:`wet` build of ``ros_tutorials``.. same launches/runs work from buildspace
* install to PREFIX, launches/runs still work.

.. rubric:: Brian/Ken

Userspace ros utilities

.. rubric:: Troy

ros_comm working under catkin, install of thunk-manifests to share/,
env variables ``CATKIN_BUILD_DIR`` and ``CATKIN_SOURCE_DIR`` set from
catkin in ``setup.sh``, whatever anybody needs.

to the left (towards the build farm)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. rubric:: Ethan and Tully

* Get catkin and necessary 3rd party binary and source debs churning
  on our build farm and posted to a deb repo.   
* Further investigate/design use of git-buildpackage for managing all
  of this; explain the architecture, workflows, pros and cons.

in the middle
^^^^^^^^^^^^^

.. rubric:: Morten

Look for gotchas in actionlib and dynamic_reconfigure, get 'em going.

Found: dynamic_reconfigure .cfg files import things from generated
python messages, but only one type from driver_base

Found: actionlib does some unnecessary calculations that introduce
buildtime dependency loops.

