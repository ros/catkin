# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Open Source Robotics Foundation, Inc. nor
#    the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior
#    written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import sys
import time

from Queue import Empty

from multiprocessing import cpu_count
from threading import Lock
try:
    # Python3
    from queue import Queue
except ImportError:
    # Python2
    from Queue import Queue

try:
    from catkin_pkg.packages import find_packages
    from catkin_pkg.topological_order import topological_order_packages
except ImportError as e:
    sys.exit(
        'ImportError: "from catkin_pkg.topological_order import '
        'topological_order" failed: %s\nMake sure that you have installed '
        '"catkin_pkg", and that it is up to date and on the PYTHONPATH.' % e
    )

from catkin.cmi.color import clr

from catkin.cmi.common import FakeLock
from catkin.cmi.common import disable_wide_log
from catkin.cmi.common import get_build_type
from catkin.cmi.common import get_cached_recursive_build_depends_in_workspace
from catkin.cmi.common import format_time_delta
from catkin.cmi.common import format_time_delta_short
from catkin.cmi.common import log
from catkin.cmi.common import wide_log

from catkin.cmi.executor import Executor
from catkin.cmi.executor import ExecutorEvent

from catkin.cmi.job import CatkinJob
from catkin.cmi.job import CMakeJob

from catkin.cmi.output import OutputController


def get_ready_packages(packages, running_jobs, completed):
    """Returns packages which have no pending depends and are ready to be built

    Iterates through the packages, seeing if any of the packages which
    are not currently in running_jobs and are not in completed jobs, have all of
    their build and buildtool depends met, and are there for ready to be queued
    up and built.

    :param packages: topologically ordered packages in the workspace
    :type packages: dict
    :param running_jobs: currently running jobs which are building packages
    :type running_jobs: dict
    :param completed: list of packages in the workspace which have been built
    :type completed: list
    :returns: list of package_path, package tuples which should be built
    :rtype: list
    """
    ready_packages = []
    workspace_packages = [(path, pkg) for path, pkg in packages]
    for path, package in packages:
        if package.name in (running_jobs.keys() + completed):
            continue
        # Collect build and buildtool depends, plus recursive build, buildtool, and run depends,
        # Excluding depends which are not in the workspace or which are completed
        uncompleted_depends = []
        depends = get_cached_recursive_build_depends_in_workspace(package, workspace_packages)
        for dep_pth, dep in depends:
            if dep.name not in completed:
                uncompleted_depends.append(dep)
        # If there are no uncompleted dependencies, add this package to the queue
        if not uncompleted_depends:
            ready_packages.append((path, package))
    # Return the new ready_packages
    return ready_packages


def queue_ready_packages(ready_packages, running_jobs, job_queue, context, force_cmake):
    """Adds packages which are ready to be built to the job queue

    :param ready_packages: packages which are ready to be built
    :type ready_packages: list
    :param running_jobs: jobs for building packages which are currently running
    :type running_jobs: dict
    :param job_queue: queue to put new jobs in, which will be consumed by executors
    :type job_queue: :py:class:`multiprocessing.Queue`
    :param context: context of the build environment
    :type context: :py:class:`catkin.cmi.context.Context`
    :param force_cmake: must run cmake if True
    :type force_cmake: bool
    :returns: updated running_jobs dict
    :rtype: dict
    """
    for path, package in ready_packages:
        build_type = get_build_type(package)
        if build_type == 'catkin':
            job = CatkinJob(package, path, context, force_cmake)
        elif build_type == 'cmake':
            job = CMakeJob(package, path, context, force_cmake)
        running_jobs[package.name] = {
            'package_number': None,
            'job': job,
            'start_time': None
        }
        job_queue.put(job)
    return running_jobs


def determine_packages_to_be_built(packages, context):
    """Returns list of packages which should be built, and those package's deps.

    :param packages: list of packages to be built, if None all packages are built
    :type packages: list
    :param context: Workspace context
    :type context: :py:class:`catkin.cmi.context.Context`
    :returns: tuple of packages to be built and those package's deps
    :rtype: tuple
    """
    start = time.time()
    workspace_packages = find_packages(context.source_space, exclude_subspaces=True)
    # If there are no packages raise
    if not workspace_packages:
        sys.exit("No packages were found in the source space '{0}'".format(context.source_space))
    log("Found '{0}' packages in {1}."
        .format(len(workspace_packages), format_time_delta(time.time() - start)))

    # Order the packages by topology
    ordered_packages = topological_order_packages(workspace_packages)
    # Set the packages in the workspace for the context
    context.packages = ordered_packages
    # Determin the packages which should be built
    packages_to_be_built = []
    packages_to_be_built_deps = []
    if packages:
        # First assert all of the packages given are in the workspace
        workspace_package_names = dict([(pkg.name, (path, pkg)) for path, pkg in ordered_packages])
        for package in packages:
            if package not in workspace_package_names:
                sys.exit("Given package '{0}' is not in the workspace".format(package))
            # If metapackage, include run depends which are in the workspace
            package_obj = workspace_package_names[package][1]
            if 'metapackage' in [e.tagname for e in package_obj.exports]:
                for rdep in package_obj.run_depends:
                    if rdep.name in workspace_package_names:
                        packages.append(rdep.name)
        # Limit the packages to be built to just the provided packages
        for pkg_path, package in ordered_packages:
            if package.name in packages:
                packages_to_be_built.append((pkg_path, package))
                # Get the recursive dependencies for each of these packages
                pkg_deps = get_cached_recursive_build_depends_in_workspace(package, ordered_packages)
                packages_to_be_built_deps.extend(pkg_deps)
    else:
        packages_to_be_built = ordered_packages
    return packages_to_be_built, packages_to_be_built_deps


def write_job_log(job_log, package, context):
    build_log_dir = os.path.join(context.workspace, 'cmi_log')
    if not os.path.exists(build_log_dir):
        os.makedirs(build_log_dir)
    package_log = os.path.join(build_log_dir, str(package) + '.log')
    with open(package_log, 'w') as f:
        f.write('\n'.join(job_log[package]))
        f.write('\n')


def build_isolated_workspace(
    context,
    packages=None,
    start_with=None,
    no_deps=False,
    jobs=None,
    force_cmake=False,
    force_color=False,
    quiet=False,
    interleave_output=False,
    no_status=False,
    lock_install=False
):
    """Builds a catkin workspace in isolation

    This function will find all of the packages in the source space, start some
    executors, feed them packages to build based on dependencies and topological
    ordering, and then monitor the output of the executors, handling loggings of
    the builds, starting builds, failing builds, and finishing builds of
    packages, and handling the shutdown of the executors when appropriate.

    :param context: context in which to build the catkin workspace
    :type context: :py:class:`catkin.cmi.context.Context`
    :param packages: list of packages to build, by default their dependencies will also be built
    :type packages: list
    :param start_with: package to start with, skipping all packages which proceed it in the topological order
    :type start_with: str
    :param no_deps: If True, the dependencies of packages will not be built first
    :type no_deps: bool
    :param jobs: number of parallel package build jobs
    :type jobs: int
    :param force_cmake: forces invocation of CMake if True, default is False
    :type force_cmake: bool
    :param force_color: forces colored output even if terminal does not support it
    :type force_color: bool
    :param quiet: suppresses the output of commands unless there is an error
    :type quiet: bool
    :param interleave_output: prints the output of commands as they are received
    :type interleave_output: bool
    :param no_status: disables status bar
    :type no_status: bool
    :param lock_install: causes executors to synchronize on access of install commands
    :type lock_install: bool

    :raises: SystemExit if buildspace is a file or no packages were found in the source space
        or if the provided options are invalid
    """
    # If no_deps is given, ensure packages to build are provided
    if no_deps and packages is None:
        sys.exit("With --no-deps, you must specify packages to build.")
    # Make sure there is a build folder and it is not a file
    if os.path.exists(context.build_space):
        if os.path.isfile(context.build_space):
            sys.exit("Build space '{0}' exists but is a file and not a folder.".format(context.build_space))
    # If it dosen't exist, create it
    else:
        log("Creating buildspace directory, '{0}'".format(context.build_space))
        os.makedirs(context.build_space)

    # Summarize the context
    log(context.summary())

    # Find list of packages in the workspace
    packages_to_be_built, packages_to_be_built_deps = determine_packages_to_be_built(packages, context)
    completed_packages = []
    if no_deps:
        # Consider deps as "completed"
        completed_packages.extend(packages_to_be_built_deps)
    else:
        # Extend packages to be built to include their deps
        packages_to_be_built.extend(packages_to_be_built_deps)
    # Also resort
    packages_to_be_built = topological_order_packages(dict(packages_to_be_built))

    # Setup pool of executors
    executors = {}
    # The communication queue can have ExecutorEvent's or str's passed into it from the executors
    comm_queue = Queue()
    # The job queue has Jobs put into it
    job_queue = Queue()
    # Lock for install space
    install_lock = Lock() if lock_install else FakeLock()
    # Determine the number of executors
    jobs = cpu_count() if jobs is None else int(jobs)
    # If only one set of jobs, turn on interleaving to get more responsive feedback
    if jobs == 1:
        # TODO: make the system more intelligent so that it can automatically switch to streaming output
        #       when only one job is building, even if multiple jobs could be building
        interleave_output = True
    # Start the executors
    for x in range(jobs):
        e = Executor(x, context, comm_queue, job_queue, install_lock)
        executors[x] = e
        e.start()

    # Variables for tracking running jobs and built/building packages
    total_packages = len(packages_to_be_built)
    package_count = 0
    running_jobs = {}
    log_dir = os.path.join(context.build_space, 'cmi_logs')
    color = True
    if not force_color and not sys.stdout.isatty():
        color = True
    out = OutputController(log_dir, quiet, interleave_output, color, prefix_output=(jobs > 1))
    if no_status:
        disable_wide_log()

    # Prime the job_queue
    ready_packages = []
    if start_with is None:
        ready_packages = get_ready_packages(packages_to_be_built, running_jobs, completed_packages)
    while start_with is not None:
        ready_packages.extend(get_ready_packages(packages_to_be_built, running_jobs, completed_packages))
        while ready_packages:
            pth, pkg = ready_packages.pop(0)
            if pkg.name != start_with:
                completed_packages.append(pkg.name)
                package_count += 1
                wide_log("[cmi] Skipping package '{0}'".format(pkg.name))
            else:
                ready_packages.insert(0, (pth, pkg))
                start_with = None
                break
    running_jobs = queue_ready_packages(ready_packages, running_jobs, job_queue, context, force_cmake)
    assert running_jobs

    error_state = False
    errors = []

    def set_error_state(error_state):
        if error_state:
            return
        # Set the error state to prevent new jobs
        error_state = True
        # Empty the job queue
        while not job_queue.empty():
            job_queue.get()
        # Kill the executors by sending a None to the job queue for each of them
        for x in range(jobs):
            job_queue.put(None)

    # While any executors are running, process executor events
    while executors:
        try:
            # Try to get an event from the communications queue
            try:
                event = comm_queue.get(True, 0.1)
            except Empty:
                # timeout occured, create null event to pass through checks
                event = ExecutorEvent(None, None, None, None)

            if event.event_type == 'job_started':
                package_count += 1
                running_jobs[event.package]['package_number'] = package_count
                running_jobs[event.package]['start_time'] = time.time()
                out.job_started(event.package)

            if event.event_type == 'command_started':
                out.command_started(event.package, event.data['cmd'], event.data['location'])

            if event.event_type == 'command_log':
                out.command_log(event.package, event.data['message'])

            if event.event_type == 'command_failed':
                out.command_failed(event.package, event.data['cmd'], event.data['location'], event.data['retcode'])
                # Add to list of errors
                errors.append(event)
                # Remove the command from the running jobs
                del running_jobs[event.package]
                # If it hasn't already been done, stop the executors
                set_error_state(error_state)

            if event.event_type == 'command_finished':
                out.command_finished(event.package, event.data['cmd'], event.data['location'], event.data['retcode'])

            if event.event_type == 'job_finished':
                completed_packages.append(event.package)
                run_time = format_time_delta(time.time() - running_jobs[event.package]['start_time'])
                out.job_finished(event.package, run_time)
                del running_jobs[event.package]
                # If shutting down, do not add new packages
                if error_state:
                    continue
                # Calculate new packages
                wide_log('[cmi] Calculating new jobs...', end='\r')
                sys.stdout.flush()
                ready_packages = get_ready_packages(packages_to_be_built, running_jobs, completed_packages)
                running_jobs = queue_ready_packages(ready_packages, running_jobs, job_queue, context, force_cmake)
                wide_log('[cmi] Uh...', end='\r')
                sys.stdout.flush()
                # Make sure there are jobs to be/being processed, otherwise kill the executors
                if not running_jobs:
                    # Kill the executors by sending a None to the job queue for each of them
                    for x in range(jobs):
                        job_queue.put(None)

            # If an executor exit event, join it and remove it from the executors list
            if event.event_type == 'exit':
                # If an executor has an exception, set the error state
                if event.data['reason'] == 'exception':
                    set_error_state(error_state)
                    errors.append(event)
                # Join and remove it
                executors[event.executor_id].join()
                del executors[event.executor_id]

            if not no_status:
                # Update the status bar on the screen
                executing_jobs = []
                for name, value in running_jobs.items():
                    number, job, start_time = value['package_number'], value['job'], value['start_time']
                    if number is None or start_time is None:
                        continue
                    executing_jobs.append({
                        'number': number,
                        'name': name,
                        'run_time': format_time_delta_short(time.time() - start_time)
                    })
                msg = "[cmi] "
                # If errors post those
                if errors:
                    for error in errors:
                        msg += clr("[!{package}] ").format(package=error.package)
                # Print them in order of started number
                for job_msg_args in sorted(executing_jobs, key=lambda args: args['number']):
                    msg += clr("[{name} - {run_time}] ").format(**job_msg_args)
                msg_rhs = clr("[{0}/{1} Active | {2}/{3} Completed]").format(
                    len(executing_jobs),
                    len(executors),
                    len(completed_packages),
                    total_packages
                )
                # Update title bar
                sys.stdout.write("\x1b]2;[cmi] {0}/{1}\x07"
                                 .format(len(completed_packages), total_packages))
                # Update status bar
                wide_log(msg, rhs=msg_rhs, end='\r')
                sys.stdout.flush()
        except KeyboardInterrupt:
            wide_log("[cmi] User interrupted, stopping.")
            set_error_state(error_state)
    # All executors have shutdown
    sys.stdout.write("\x1b]2;\x07")
    if not errors:
        wide_log("[cmi] Finished.")
        return 0
    else:
        wide_log(clr("[cmi] There were @!@{rf}errors@|:"))
        for error in errors:
            if error.event_type == 'exit':
                wide_log("""Executor '{exec_id}' had an unhandle exception while processing package '{package}':

{data[exc]}
""".format(exec_id=error.executor_id + 1, **error.__dict__))
            else:
                wide_log(clr("""
@{rf}Failed@| to build package '@{cf}{package}@|' because the following command:

    @!@{kf}# Command run in directory: @|{location}
    {cmd.cmd_str}

@{rf}Exited@| with return code: @!{retcode}@|""").format(package=error.package, **error.data))
        sys.exit(1)
