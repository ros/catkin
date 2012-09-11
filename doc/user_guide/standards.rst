CMake coding standards
======================

**Use ${PROJECT_NAME} wherever possible**

Use ``${PROJECT_NAME}`` instead of repeating the project name
manually.


**find_package(... REQUIRED)**

Use ``REQUIRED`` on all calls to ``find_package`` if they aren't
actually optional (i.e. you're not going to check ``thing_FOUND``
and enable/disable features).


Code style
----------

**Keep lists *sorted***

Whenever using a list of items (i.e. in find_package(COMPONENTS ...)
or files which should be build or installed) keep them alphabetically
sorted.  This improves readability when looking for specific items.
(There are exceptions which require a specific custom order like the
list of projects inside a stack).

**Lowercase keywords**

Keywords like ``if``, ``for`` etc. are all lowercase.


**Closing keyword should have empty parenthesis**

The closing keywords like ``endif()`` and ``endforeach()`` should not repeat the condition of the opening keyword.
