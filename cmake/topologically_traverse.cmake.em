@{
import apt_pkg, os, sys

class Pkg:
    def __init__(self, **kwds):
        self.__dict__.update(path=None, versioned_depends=None, genlang=None, depends=None)
        self.__dict__.update(kwds)
    def __repr__(self):
        return "%s %s %s %s\n" % (self.path, self.versioned_depends, self.depends, self.genlang)

pkgs = {}

PKG_NAME_FIELD = "X-ROS-Pkg-Name"
PKG_DEP_FIELD = "X-ROS-Pkg-Depends"
SYS_DEP_FIELD = "X-ROS-System-Depends"
GENERATOR_FIELD = "X-ROS-Message-Generator"

def read_control_file(source_dir, control_file):
    tag_file = apt_pkg.TagFile(open(control_file))

    for sec in tag_file:
        if PKG_NAME_FIELD in sec:
            pkg_name = sec[PKG_NAME_FIELD]
            if enabled_projects[0] != 'ALL' and pkg_name not in enabled_projects:
                break
            pkgs[pkg_name] = p = Pkg(path=source_dir)
            p.versioned_depends = apt_pkg.parse_depends(sec[PKG_DEP_FIELD])
            p.depends = set([d[0][0] for d in p.versioned_depends])
            p.genlang = sec.get(GENERATOR_FIELD, None)
            break

    else:
        print 'MESSAGE(STATUS "%s missing %s")'%(control_file, PKG_NAME_FIELD)

#def find_external_pkgs(pkgs):
#    return reduce( set.union, pkgs.values()) - set(dep_map.keys())

def remove_deps(pkgs, name):
    for dep_list in [p.depends for p in pkgs.values()]:
        dep_list.difference_update([name])

def topo_packages_generators_first(pkgs):
    """
    First return packages which have generators. Then return the rest.
    """
    while len(pkgs) > 0:
        for name, p in pkgs.items():
            if p.genlang and not p.depends:
                remove_deps(pkgs, name)
                del pkgs[name]
                yield name
                break
        for name, p in pkgs.items():
            if not p.depends:
                remove_deps(pkgs, name)
                del pkgs[name]
                yield name
                break

for dirpath,dirnames,filenames in os.walk(source_root_dir):
    if 'debian' in dirnames and not os.path.basename(dirpath).startswith('.'):
        control_file = dirpath+"/debian/control"
        if os.path.exists(control_file + ".em"):
            read_control_file(dirpath, control_file + ".em")
        elif os.path.exists(control_file):
            read_control_file(dirpath, control_file)
        dirnames[:] = [] #stop walk

all_deps = reduce(set.union, [p.depends for p in pkgs.values()])

unknown_deps = all_deps - set(pkgs)

del pkgs['catkin']
remove_deps(pkgs, 'catkin')

if len(unknown_deps) > 0:
    print 'message(FATAL_ERROR "\nUnknown dependencies: %s\n")' % ' '.join(unknown_deps)
    topo_pkgs = []
else:
    topo_pkgs = topo_packages_generators_first(pkgs)
}

message(STATUS "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
message(STATUS "~v^V^v~   traversing stacks/projects in topological order   ~v^V^v~")
message(STATUS "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
set(ROS_LANGS genpy gencpp)

@[for pkgname in topo_pkgs]
message(STATUS "+++ @pkgname")
add_subdirectory(@(pkgname))
@[end for]
