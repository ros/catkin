Environment files
=================

.. rubric:: In the buildspace

(where ``CMAKE_SOURCE_DIR=/tmp/src`` and ``CMAKE_BINARY_DIR=/tmp/src/build``)::

  % cat setup.sh 
  export PYTHONPATH=/tmp/src/build/gen/py:$PYTHONPATH
  export PATH=/tmp/src/build/bin:$PATH
  export CATKIN_BINARY_DIR=/tmp/src/build
  export CATKIN_SOURCE_DIR=/tmp/src

.. rubric:: Installed

(where ``CMAKE_INSTALL_PREFIX=/opt/ros/fuerte``)::

  % cat /tmp/PREFIX/setup.sh 
  #
  # PREFIX/lib/pythonX.Y/dist-packages is the way it looks under --install-layout=deb
  #
  
  PYTHONPATH=/tmp/PREFIX/lib/python2.7/dist-packages:$PYTHONPATH
  export PYTHONPATH
  
  

There is a file ``env.sh`` that takes a variable number of arguments.
It will load the ``setup.sh`` environment and execute its arguments in
a subshell... this is to allow ``setup.sh`` to be the single point of
environment definition.




