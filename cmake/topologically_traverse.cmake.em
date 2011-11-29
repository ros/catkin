@{
import yaml, os, sys, glob

class Pkg:
    def __init__(self, **kwds):
        self.__dict__.update(path=None, genlang=None, depends=None)
        self.__dict__.update(kwds)
    def __repr__(self):
        return "path=%s depends=%s genlang=%s\n" % (self.path, self.depends, self.genlang)

pkgs = {}

PKG_NAME_FIELD = "Catkin-ProjectName"
PKG_DEP_FIELD = "Catkin-Depends"
GENERATOR_FIELD = "Catkin-ROS-Message-Generator"

def read_control_file(source_dir, control_file):
    stackyaml = yaml.load(open(control_file))

    pkg_name = stackyaml[PKG_NAME_FIELD]
    # print >>sys.stderr, "pkg_name=<%s>" % pkg_name
    if enabled_projects[0] != 'ALL' and pkg_name not in enabled_projects:
        return
    pkgs[pkg_name] = p = Pkg(path=source_dir)
    p.depends = stackyaml.get('Catkin-Depends', '')
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
                return name
        for name, p in pkgs.items():
            if not p.depends:
                return name
        return None
    
    while len(pkgs) > 0:
        name = next_pkg()
        # print >>sys.stderr, "name=", name
        del pkgs[name]
        remove_deps(pkgs, name)
        yield name

stackyamls = glob.glob(os.path.join(source_root_dir, '*', 'stack.yaml'))
# print >>sys.stderr, "stackyamls=", stackyamls
for stackyamlfilename in stackyamls:
    if os.path.basename(os.path.dirname(stackyamlfilename)).startswith('.'):
        continue
    read_control_file(os.path.dirname(stackyamlfilename), stackyamlfilename)

all_deps = reduce(set.union, [p.depends for p in pkgs.values()])

unknown_deps = all_deps - set(pkgs)

del pkgs['catkin']
remove_deps(pkgs, 'catkin')

# print >>sys.stderr, "len=", len(unknown_deps), unknown_deps
langs = [name for name,p in pkgs.items() if p.genlang]

if len(unknown_deps) > 0:
    print 'message(FATAL_ERROR "\nUnknown dependencies: <%s>\n")' % '|'.join(unknown_deps)
    topo_pkgs = []
else:
    topo_pkgs = topo_packages_generators_first(pkgs)
}

message(STATUS "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
message(STATUS "~v^V^v~   traversing stacks/projects in topological order   ~v^V^v~")
message(STATUS "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
set(CATKIN_GENLANGS @(' '.join(langs)))

@[for pkgname in topo_pkgs]
message(STATUS "+++ @pkgname")
stamp(@(pkgname)/stack.yaml)
add_subdirectory(@(pkgname))
@[end for]
