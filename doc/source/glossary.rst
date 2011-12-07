Glossary
--------

.. glossary::

   dry
        a (package/stack/whatever) not yet converted to catkin/FHS build & install.

   wet
        a (package/stack/whatever) stack not yet converted to catkin/FHS build & install.

   stack
        unit of installation.  These get made in to debs.

   package 
        generally something "smaller" than a stack... but a package
        can be a stack.

   project
        CMake's notion of buildable subdirectory: it contains a
        ``CMakeLists.txt`` that calls CMake's ``project()`` macro.  These can next.

   generated code
        Code generated during the build process, typically by a message code generator package.  
        May or may not require compilation.

   static code
        Code typed in by a developer, contrast :term:`generated code`        


