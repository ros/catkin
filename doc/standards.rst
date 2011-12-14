CMake coding standards
----------------------

* use REQUIRED on your calls to find_package if they aren't actually
  optional (i.e. you're not going to check ``thing_FOUND`` and
  enable/disable features)




Optional features
-----------------

.. highlight:: cmake

Optional functionality that depend on user-provided values should be
enabled via cache variables that are set to OFF by default, e.g.::

  set(CATKIN_DOCS_DEPLOY_DESTINATION "OFF" CACHE STRING
    "Deploy destination for docs, or OFF.  Will be passed to rsync; may contain user@ syntax for ssh"
    )

  [later in cmakelists]

  if(CATKIN_DOCS_DEPLOY_DESTINATION)
    add_custom_target(sphinx-deploy)
  endif()

