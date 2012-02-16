CMake coding standards
----------------------

Use ${PROJECT_NAME} wherever possible
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use ``${PROJECT_NAME}`` instead of repeating the project name
manually.


find_package(... REQUIRED)
^^^^^^^^^^^^^^^^^^^^^^^^^^

Use ``REQUIRED`` on all calls to ``find_package`` if they aren't
actually optional (i.e. you're not going to check ``thing_FOUND``
and enable/disable features).


Keep lists *sorted*
^^^^^^^^^^^^^^^^^^^

Whenever using a list of items (i.e. in find_package(COMPONENTS ...)
or files which should be build or installed) keep them alphabetically
sorted.  This improves readability when looking for specific items.
(There are exceptions which require a specific custom order like the
list of projects inside a stack).


Optional features
^^^^^^^^^^^^^^^^^

Optional functionality that depends on user-provided values should be
enabled via cache variables that are set to ``OFF`` by default, e.g.::

  set(CATKIN_DOCS_DEPLOY_DESTINATION "OFF" CACHE STRING
    "Deploy destination for docs, or OFF.  Will be passed to rsync; may contain user@ syntax for ssh"
    )

  [later in CMakeLists.txt]

  if(CATKIN_DOCS_DEPLOY_DESTINATION)
    add_custom_target(sphinx-deploy)
  endif()

