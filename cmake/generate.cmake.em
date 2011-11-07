@{
import apt_pkg
import os

pkg_dep_map = {}
pkg_dep_versions = {}
pkg_dirs = {}

PKG_NAME_FIELD = "X-ROS-Pkg-Name"
PKG_DEP_FIELD = "X-ROS-Pkg-Depends"
SYS_DEP_FIELD = "X-ROS-System-Depends"

def read_control_file(source_dir, control_file):
    tag_file = apt_pkg.TagFile(open(control_file))
    
    for sec in tag_file:
        if PKG_NAME_FIELD in sec:
            pkg_name = sec[PKG_NAME_FIELD]
            pkg_dirs[pkg_name] = source_dir

            # full deps with version numbers
            pkg_dep_versions[pkg_name] = apt_pkg.parse_depends(sec[PKG_DEP_FIELD])

            # create a list with only names
            pkg_dep_map[pkg_name] = set()
            for d in pkg_dep_versions[pkg_name]:
                pkg_dep_map[pkg_name].add(d[0][0])

            break
    else:
        print 'MESSAGE(STATUS "%s missing %s")'%(control_file, PKG_NAME_FIELD)

def find_external_pkgs(dep_map):
    return reduce( set.union, dep_map.values()) - set(dep_map.keys())

def remove_deps(dep_map, to_remove):
    for dep_list in dep_map.values():
        dep_list.difference_update(to_remove)

def find_pkgs_with_no_deps(dep_map):
    return set(pkg for pkg,deps in dep_map.items() if not deps)

def topological_sort_generator(dep_map):
    nodeps = find_pkgs_with_no_deps(dep_map)
    while(nodeps):
        remove_deps(dep_map, nodeps)
        for pkg in nodeps:
            yield pkg
            del dep_map[pkg]
        nodeps = find_pkgs_with_no_deps(dep_map)

for dirpath,dirnames,filenames in os.walk(source_root_dir):
    #print dirpath
    if 'debian' in dirnames:
        control_file = dirpath+"/debian/control"
        if os.path.exists(control_file):
            read_control_file(dirpath, control_file)
        dirnames[:] = [] #stop walk

# check unsatisfied deps
#print "pkg_dep_map"
#print pkg_dep_map
external = find_external_pkgs(pkg_dep_map)

for ext in external:
    print 'MESSAGE(STATUS " - put check for external dep %s - ")'%ext

remove_deps(pkg_dep_map, external)

for pkg in topological_sort_generator(pkg_dep_map):
    print 'MESSAGE(STATUS "******* %s ******")'%pkg
    print "add_subdirectory(%s)"%pkg_dirs[pkg]

}
