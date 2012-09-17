from __future__ import print_function
import glob
import os
import rospkg.stack
import sys


class ProjectData:

    def __init__(self, xml_filename, rospkg_arg=rospkg):
        project = rospkg_arg.stack.parse_stack_file(xml_filename)
        self.name = project.name
        self.path = os.path.dirname(xml_filename)
        self.build_depends = set([d.name for d in project.build_depends])
        self.message_generator = project.message_generator

    def __repr__(self):
        return 'name=%s path=%s build_depends=%s message_generator=%s\n' % (self.name, self.path, self.build_depends, self.message_generator)


def _remove_dependency(projects, name):
    for build_depends in [project_data.build_depends for project_data in projects.values()]:
        if build_depends is not None:
            build_depends.difference_update([name])


def _sort_projects(projects):
    '''
    First returning projects which have message generators and then the rest based on their build_depends.
    '''

    ordered_projects = []
    while len(projects) > 0:
        # find all projects without build dependencies
        message_generators = []
        non_message_generators = []
        for name, data in projects.items():
            if not data.build_depends:
                if data.message_generator:
                    message_generators.append(name)
                else:
                    non_message_generators.append(name)
        # first choose message generators
        if message_generators:
            names = message_generators
        elif non_message_generators:
            names = non_message_generators
        else:
            # in case of a circular dependency pass the list of remaining projects
            ordered_projects.append([None, ', '.join(sorted(projects.keys()))])
            break

        # alphabetic order only for convenience
        names.sort()
        #print('names = %s' % names, file=sys.stderr)

        # add first candidates to ordered list
        # do not add all candidates since removing the depends from the first might affect the next candidates
        name = names[0]
        ordered_projects.append([name, projects[name]])
        # remove project from further processing
        del projects[name]
        _remove_dependency(projects, name)
    return ordered_projects


def topological_order(source_root_dir, whitelisted=None, blacklisted=None):
    xml_filenames = glob.glob(os.path.join(source_root_dir, '*', 'stack.xml'))
    #print('xml_filenames = %s' % xml_filenames, file=sys.stderr)

    # fetch all meta data
    prefix = os.path.abspath(source_root_dir) + os.sep
    project_data_list = []
    for xml_filename in xml_filenames:
        if os.path.basename(os.path.dirname(xml_filename)).startswith('.'):
            continue
        data = ProjectData(xml_filename)
        # make path relative to root dir
        if data.path.startswith(prefix):
            data.path = data.path[len(prefix):]
        project_data_list.append(data)
    return _topological_order_projects(project_data_list, whitelisted, blacklisted)

def _topological_order_projects(project_data_list, whitelisted=None, blacklisted=None):
    projects = {}
    for data in project_data_list:
        # skip non-whitelisted projects
        if whitelisted and data.name not in whitelisted:
            continue
        # skip blacklisted projects
        if blacklisted and data.name in blacklisted:
            continue
        if data.name in projects:
            print('Two stacks with the same name "%s" in the workspace:\n- %s\n- %s' % (data.name, projects[data.name].path, data.path), file=sys.stderr)
            sys.exit(1)
        projects[data.name] = data

    # remove catkin from list of projects
    if 'catkin' in projects:
        del projects['catkin']

    # remove external dependencies from the list
    if projects:
        all_build_depends = reduce(set.union, [p.build_depends for p in projects.values()])
        external_depends = all_build_depends - set(projects.keys())
        #print('external_depends = %s' % external_depends, file=sys.stderr)
        for data in projects.values():
            data.build_depends = set(data.build_depends) - set(external_depends)

    return _sort_projects(projects)


def get_message_generators(ordered_projects):
    return [name for (name, data) in ordered_projects if data.message_generator]
