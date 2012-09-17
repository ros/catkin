How to build the docs
=====================

catkin provides (and uses) some plugins to `Sphinx
<http://sphinx.pocoo.org/>`_ to build documentation.  These plugins can
be used to provide a common look and feel to the generated documentation.

The first time you want to build catkin-controlled documentation (including
catkin's own documentation), you'll need to setup your environment.

Setup
-----

#. Get ``catkin-sphinx``::

       git clone git://github.com/ros/catkin-sphinx.git

#. Build and install it::

       cd catkin-sphinx
       sudo python setup.py install

#. Copy ``ros-theme`` to your Sphinx configuration directory::

       cp -a theme/ros-theme ~/sphinx

Now you can build documentation for projects that use the ``ros-theme``.  For
example, to build catkin's documentation::
 
    git clone git://github.com/ros/catkin.git
    cd catkin/doc
    make html

Usage
-----

To use the ``ros-theme`` in your own project's documentation, add the
following line to your ``conf.py``::

    extensions = extensions + ['catkin_sphinx.cmake', 'catkin_sphinx.ShLexer']

ros-theme-enabled projects use the following ``html_theme_path`` setting::

    html_theme_path = [os.path.join(os.path.expanduser('~'), 'sphinx'), 'themes']
