@{
import yaml, os, sys, glob, em

class Pkg:
    def __init__(self, **kwds):
        self.__dict__.update(path=None, genlang=None, depends=None)
        self.__dict__.update(kwds)
    def __repr__(self):
        return "path=%s depends=%s genlang=%s\n" % (self.path, self.depends, self.genlang)

pkgs = {}

PKG_NAME_FIELD = "Catkin-ProjectName"
PKG_DEP_FIELD = "Depends"
GENERATOR_FIELD = "Catkin-ROS-Message-Generator"

def read_control_file(source_dir, control_file):
    stackyaml = yaml.load(open(control_file))

    pkg_name = stackyaml[PKG_NAME_FIELD]
    # print >>sys.stderr, "pkg_name=<%s>" % pkg_name
    if 'ALL' not in enabled_stacks and pkg_name not in enabled_stacks:
        return
    if pkg_name in blacklisted_stacks:
        return
    pkgs[pkg_name] = p = Pkg(path=source_dir.replace("\\","/"))
    p.depends = stackyaml.get('Depends', '')
    if type(p.depends) == str:
        p.depends = set([x.strip() for x in p.depends.split(',') if len(x.strip()) > 0])
    else:
        p.depends = set([])
    p.genlang = stackyaml.get(GENERATOR_FIELD, None)
    # print >>sys.stderr, "p=", p

#def find_external_pkgs(pkgs):
#    return reduce( set.union, pkgs.values()) - set(dep_map.keys())

def remove_deps(pkgs, name):
    for dep_list in [p.depends for p in pkgs.values()]:
        dep_list.difference_update([name])

def topo_packages_generators_first(pkgs):
    """
    First return packages which have generators. Then return the rest.
    """

    def next_pkg():
        for name, p in pkgs.items():
            if p.genlang and not p.depends:
                return (name,p)
        for name, p in pkgs.items():
            if not p.depends:
                return (name,p)
        return (None, None)

    topo_pkgs = []
    while len(pkgs) > 0:
        (name,p) = next_pkg()
        if name is None:
            # in case of a circular dependency pass the list of remaining packages
            topo_pkgs.append([None, ', '.join(pkgs.keys())])
            break
        del pkgs[name]
        remove_deps(pkgs, name)
        topo_pkgs.append([name,p])
    return topo_pkgs

stackyamls = glob.glob(os.path.join(source_root_dir, '*', 'stack.yaml'))
# print >>sys.stderr, "stackyamls=", stackyamls
for stackyamlfilename in stackyamls:
    if os.path.basename(os.path.dirname(stackyamlfilename)).startswith('.'):
        continue
    read_control_file(os.path.dirname(stackyamlfilename), stackyamlfilename)
if pkgs:
    all_deps = reduce(set.union, [p.depends for p in pkgs.values()])
else:
    all_deps = set()

unknown_deps = all_deps - set(pkgs)

#remove unknown dependencies from the list
for p in pkgs.values():
    p.depends = set(p.depends) - set(unknown_deps)

if 'catkin' in pkgs:
    del pkgs['catkin']
    remove_deps(pkgs, 'catkin')

langs = [name for name,p in pkgs.items() if p.genlang]

topo_pkgs = topo_packages_generators_first(pkgs)
}

message(STATUS "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
message(STATUS "~~         traversing stacks/projects in dependency order         ~~")
message(STATUS "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
set(CATKIN_GENLANGS @(' '.join(langs)))

@[for name,pkg in topo_pkgs]
@[if not name]
message(FATAL_ERROR "Circular dependency in subset of packages:\n@pkg")
@[end if]
@[end for]

@[for name,pkg in topo_pkgs]
@[if name]
message(STATUS "+++ @name")
set(CATKIN_CURRENT_STACK "" CACHE INTERNAL "" FORCE)
stamp(@(pkg.path)/stack.yaml)
add_subdirectory(@(pkg.path))
@[end if]
@[end for]
