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
from multiprocessing import Queue

try:
    from catkin_pkg.packages import find_packages
    from catkin_pkg.topological_order import topological_order_packages
except ImportError as e:
    sys.exit(
        'ImportError: "from catkin_pkg.topological_order import '
        'topological_order" failed: %s\nMake sure that you have installed '
        '"catkin_pkg", and that it is up to date and on the PYTHONPATH.' % e
    )

from catkin.cmi.common import get_build_type
from catkin.cmi.common import get_cached_recursive_build_depends_in_workspace
from catkin.cmi.common import format_time_delta
from catkin.cmi.common import log
from catkin.cmi.common import wide_log

from catkin.cmi.executor import Executor

from catkin.cmi.job import CatkinJob
from catkin.cmi.job import CMakeJob


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
    log("Found '{0}' packages in '{1}'."
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
        workspace_package_names = [pkg.name for path, pkg in ordered_packages]
        for package in packages:
            if package not in workspace_package_names:
                sys.exit("Given package '{0}' is not in the workspace".format(package))
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


def build_isolated_workspace(
    context,
    packages=None,
    start_with=None,
    no_deps=False,
    jobs=None,
    force_cmake=False,
    force_color=False,
    quiet=False,
    interleave_output=False
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

    # Setup pool of executors
    executors = {}
    # The communication queue can have ExecutorEvent's or str's passed into it from the executors
    comm_queue = Queue()
    # The job queue has Jobs put into it
    job_queue = Queue()
    # Determine the number of executors
    jobs = cpu_count() if jobs is None else int(jobs)
    # If only one set of jobs, turn on interleaving to get more responsive feedback
    if jobs == 1:
        # TODO: make the system more intelligent so that it can automatically switch to streaming output
        #       when only one job is building, even if multiple jobs could be building
        interleave_output = True
    # Start the executors
    for x in range(jobs):
        e = Executor(x, context, comm_queue, job_queue)
        executors[x] = e
        e.start()

    # Variables for tracking running jobs and built/building packages
    total_packages = len(packages_to_be_built)
    package_count = 0
    running_jobs = {}
    command_log_cache = {}

    # Prime the job_queue
    ready_packages = get_ready_packages(packages_to_be_built, running_jobs, completed_packages)
    running_jobs = queue_ready_packages(ready_packages, running_jobs, job_queue, context, force_cmake)
    assert running_jobs

    error_state = False
    errors = []

    # While the executors are all running, process executor events
    while executors:
        # Try to get data from the communications queue
        try:
            item = comm_queue.get(True, 0.1)
            # If a log event, print it
            if item.event_type == 'log':
                wide_log('[cmi-{executor_id}]: {message}'.format(**item.__dict__))

            # If a command log, cache it and potentially print it out
            if item.event_type == 'command_log':
                msg = '[{package}]: {message}'.format(**item.__dict__)
                assert item.package in command_log_cache, "command log received before command started"
                command_log_cache[item.package].append(msg)
                if not quiet and interleave_output:
                    if jobs == 1:
                        wide_log(msg[len('[{0}]: '.format(item.package)):])
                    else:
                        wide_log(msg)

            # If a command started, print message
            if item.event_type == 'command_started':
                msg = "[{package}]: ==> '{message[0]}' in '{message[1]}'".format(**item.__dict__)
                assert item.package not in command_log_cache, "command log cache entry exists, this should not happen"
                command_log_cache[item.package] = []
                command_log_cache[item.package].append(msg)
                wide_log(msg)

            # If a command finished or failed and interleave_output is off, print the cached output
            if item.event_type in ['command_finished', 'command_failed'] and not interleave_output and not quiet:
                assert item.package in command_log_cache, "command finished or failed before starting"
                if len(command_log_cache[item.package]) > 1:
                    wide_log("")  # Empty line
                    wide_log("OUT [{package}]:".format(**item.__dict__))
                    for index, line in enumerate(command_log_cache[item.package]):
                        if index == 0:
                            # Skip the first line (started message)
                            continue
                        wide_log(line[len('[{0}]: '.format(item.package)):])

            # If a command finished, print message
            if item.event_type == 'command_finished':
                msg = ("[{package}]: <== Command '{message[0]}' finished with exit code '{message[1]}'"
                       .format(**item.__dict__))
                assert item.package in command_log_cache, "command finished before starting"
                command_log_cache[item.package].append(msg)
                if not quiet:
                    wide_log(msg)

            # If a command failed, shutdown everything
            if item.event_type == 'command_failed':
                msg = ("<== Failed to build {package}, command '{message[0]}' exited with return code '{message[1]}'"
                       .format(**item.__dict__))
                assert item.package in command_log_cache, "command failed before starting"
                command_log_cache[item.package].append(msg)
                # If quiet was on, print the error, as it has not been printed before
                if quiet:
                    for index, line in enumerate(command_log_cache[item.package]):
                        if index == 0:
                            # Skip the first line (started message)
                            continue
                        wide_log(line)
                wide_log(msg)
                errors.append(item)
                # Remove the command from the running jobs
                del running_jobs[item.package]
                if not error_state:
                    # Dispatch kill signal to executors
                    for x in range(jobs):
                        job_queue.put(None)
                    # Change loop behavior to prevent new jobs
                    error_state = True

            # If a command finished or failed, clear the command log cache
            if item.event_type in ['command_finished', 'command_failed']:
                assert item.package in command_log_cache, "command finished or failed before starting"
                # TODO: log to a file? or store as part of bigger job cache?
                del command_log_cache[item.package]

            # If an executor exit event, join it and remove it from the executors list
            if item.event_type == 'exit':
                executors[item.executor_id].join()
                del executors[item.executor_id]

            # If a job started event, assign it a number and a start time
            if item.event_type == 'job_started':
                package_count += 1
                running_jobs[item.package]['package_number'] = package_count
                running_jobs[item.package]['start_time'] = time.time()
                wide_log("==> Starting build of package '{0}'".format(item.package))

            # If a job finished event, remove from running_jobs, move to completed, call queue_new_jobs
            if item.event_type == 'job_finished':
                completed_packages.append(item.package)
                wide_log("<== Finished building package '{0}', it took {1}".format(
                    item.package,
                    format_time_delta(time.time() - running_jobs[item.package]['start_time']))
                )
                del running_jobs[item.package]
                # If shutting down, do not add new packages
                if error_state:
                    continue
                ready_packages = get_ready_packages(packages_to_be_built, running_jobs, completed_packages)
                running_jobs = queue_ready_packages(ready_packages, running_jobs, job_queue, context, force_cmake)
                # Make sure there are jobs to be/being processed, otherwise kill the executors
                if not running_jobs:
                    # Kill the executors by sending a None to the job queue for each of them
                    for x in range(jobs):
                        job_queue.put(None)
        except Empty:
            # timeout occured
            item = None
        except KeyboardInterrupt:
            wide_log("[cmi] User interrupted, stopping.")
            break
        finally:
            # Update the status bar on the screen
            executing_jobs = []
            for name, value in running_jobs.items():
                number, job, start_time = value['package_number'], value['job'], value['start_time']
                if number is None or start_time is None:
                    continue
                executing_jobs.append((number, total_packages, name, format_time_delta(time.time() - start_time)))
            msg = "[cmi {0}/{1}] ".format(len(executing_jobs), jobs)
            # If errors post those
            if errors:
                for error in errors:
                    msg += "[!{0}] ".format(error.package)
            # Print them in order of started number
            for job_msg_args in sorted(executing_jobs, key=lambda args: args[0]):
                msg += "[{0}/{1} {2} - {3}] ".format(*job_msg_args)
            # Update title bar
            job_numbers = [x[0] for x in executing_jobs]
            if job_numbers:
                sys.stdout.write("\x1b]2;[cmi] {0}/{1}\x07".format(max(job_numbers), total_packages))
            else:
                sys.stdout.write("\x1b]2;[cmi]\x07")
            # Update status bar
            wide_log(msg, truncate=True, end='\r')
            sys.stdout.flush()
    # All executors have shutdown
    if not errors:
        wide_log("[cmi] Finished.")
    else:
        wide_log("[cmi] There were errors:")
        for error in errors:
            wide_log(("""
Failed to build package '{package}' becuase the following command:

    # Command run in '{message[2]}' directory
    {message[0]}

Exited with return code: {message[1]}""").format(**error.__dict__))
