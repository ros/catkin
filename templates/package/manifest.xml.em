@# This is a template for a manifest.xml in a catkin package
@# See empy -H for the templating syntax
@{
import em
}@
<package>
  <description brief="@empy.CATKIN_PACKAGE.description_brief">@empy.CATKIN_PACKAGE.description</description>
  <author>@empy.CATKIN_PACKAGE.authors</author>
  <license>@empy.CATKIN_PACKAGE.license_name</license>
  <review status=""/>
  <url></url>
@{for package in empy.CATKIN_PACKAGE.dependencies:
  print('  <depend package="%s"/>' % package)
}@
  <export>
    <cpp cflags="`pkg-config --cflags @empy.CATKIN_PACKAGE.name`"
         lflags="`pkg-config --libs @empy.CATKIN_PACKAGE.name`"/>
    <roslang cmake="${prefix}/rosbuild/@(empy.CATKIN_PACKAGE.name).cmake"/>
  </export>
  <rosdep name="pkg-config"/>
</package>
