<package>
  <description brief="@PACKAGE_NAME">

  </description>

  <author>FIXME</author>
  <license>BSD</license>
  <url>FIXME</url>

  @[for pkg in PACKAGE_DEPENDS.split(';')]
  <depend package="@pkg" />
  @[end for]

  <export>
    <cpp cflags="`pkg-config --cflags @PACKAGE_NAME`" 
         lflags="`pkg-config --libs @PACKAGE_NAME`" />
  </export>
</package>
