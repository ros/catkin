# generated from catkin/cmake/em/order_packages.cmake.em
@{
import os
from catkin.topological_order import get_message_generators, topological_order
ordered_packages = topological_order(os.path.normpath(source_root_dir), whitelisted_packages, blacklisted_packages)
}@

set(CATKIN_ORDERED_PACKAGES "")
set(CATKIN_ORDERED_PACKAGE_PATHS "")
set(CATKIN_ORDERED_PACKAGES_IS_META "")
@[for name, package_data in ordered_packages]@
@[if not name]@
message(FATAL_ERROR "Circular dependency in subset of packages:\n@package_data")
@[end if]@
list(APPEND CATKIN_ORDERED_PACKAGES "@(name)")
list(APPEND CATKIN_ORDERED_PACKAGE_PATHS "@(package_data.path.replace(os.sep, '/'))")
list(APPEND CATKIN_ORDERED_PACKAGES_IS_META "@(str(package_data.is_metapackage))")
@[end for]@

@{
message_generators = get_message_generators(ordered_packages)
}@
set(CATKIN_MESSAGE_GENERATORS @(' '.join(message_generators)))
