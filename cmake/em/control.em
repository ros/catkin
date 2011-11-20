Source: @(CATKIN_PACKAGE_PREFIX)@(Package)
Section: @(Section)
Priority: @(Priority)
Maintainer: @(Maintainer)
Build-Depends: @(empy.expand(locals()['Build-Depends'], locals()))
Homepage: @(Homepage)

Package: @(CATKIN_PACKAGE_PREFIX)@(Package)
Architecture: @(Architecture)
Depends: @(empy.expand(locals()['Depends'], locals()))
Description: @(Description)

