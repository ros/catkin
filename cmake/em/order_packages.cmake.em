# generated from catkin/cmake/em/order_packages.cmake.em
@{
import os
try:
    from catkin_pkg.topological_order import topological_order
except ImportError as impe:
    raise RuntimeError("ERROR Cannot find a module of catkin_pkg, make sure it is up to date and on the PYTHONPATH, see catkin install instructions: %s" % impe)
# vars defined in order_packages.context.py.in
ordered_packages = topological_order(os.path.normpath(source_root_dir), whitelisted_packages, blacklisted_packages)
fatal_error = False
}@

set(CATKIN_ORDERED_PACKAGES "")
set(CATKIN_ORDERED_PACKAGE_PATHS "")
set(CATKIN_ORDERED_PACKAGES_IS_META "")
@[for path, package in ordered_packages]@
@[if path is None]@
message(FATAL_ERROR "Circular dependency in subset of packages:\n@package")
@{
fatal_error = True
}@
@[elif package.name != 'catkin']@
list(APPEND CATKIN_ORDERED_PACKAGES "@(package.name)")
list(APPEND CATKIN_ORDERED_PACKAGE_PATHS "@(path)")
list(APPEND CATKIN_ORDERED_PACKAGES_IS_META "@(str('metapackage' in [e.tagname for e in package.exports]))")
@[end if]@
@[end for]@

@[if not fatal_error]@
@{
message_generators = [package.name for (_, package) in ordered_packages if 'message_generator' in [e.tagname for e in package.exports]]
}@
set(CATKIN_MESSAGE_GENERATORS @(' '.join(message_generators)))
@[end if]@
