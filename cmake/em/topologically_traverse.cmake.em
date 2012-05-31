@{
import os, sys, glob, em
import rospkg.stack

class Pkg:
    def __init__(self, **kwds):
        self.__dict__.update(path=None, genlang=None, depends=None)
        self.__dict__.update(kwds)
    def __repr__(self):
        return "path=%s depends=%s genlang=%s\n" % (self.path, self.depends, self.genlang)

pkgs = {}

def read_control_file(source_dir, control_file):
    stack = rospkg.stack.parse_stack_file(control_file)

    pkg_name = stack.name
    # print >>sys.stderr, "pkg_name=<%s>" % pkg_name
    if 'ALL' not in enabled_stacks and pkg_name not in enabled_stacks:
        return
    if pkg_name in blacklisted_stacks:
        return
    pkgs[pkg_name] = p = Pkg(path=source_dir.replace("\\","/"))
    p.depends = set([d.name for d in stack.build_depends])
    p.genlang = stack.message_generator
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

stackxmls = glob.glob(os.path.join(source_root_dir, '*', 'stack.xml'))
# print >>sys.stderr, "stackxmls=", stackxmls
for stackxmlfilename in stackxmls:
    if os.path.basename(os.path.dirname(stackxmlfilename)).startswith('.'):
        continue
    read_control_file(os.path.dirname(stackxmlfilename), stackxmlfilename)
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

langs = [name for name, p in pkgs.items() if p.genlang]

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
stamp(@(pkg.path)/stack.xml)
add_subdirectory(@(pkg.path))
@[end if]
@[end for]
