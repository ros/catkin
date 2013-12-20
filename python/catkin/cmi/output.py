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

from __future__ import print_function

import os

from catkin.cmi.common import remove_ansi_escape
from catkin.cmi.common import wide_log


class FileBackedLogCache(object):
    def __init__(self, package_name, log_dir, color):
        self.package = package_name
        self.log_dir = log_dir
        self.color = color
        self.log_path = os.path.join(log_dir, self.package + '.log')
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        self.file_handle = open(self.log_path, 'w')
        self.current_cmd = None

    def start_command(self, cmd, msg):
        self.current_cmd = cmd
        self.append(msg)

    def append(self, msg):
        self.file_handle.write(msg + '\n')
        self.file_handle.flush()

    def finish_command(self, msg):
        self.current_cmd = None
        self.append(msg)

    def close(self):
        self.file_handle.close()
        self.file_handle = None

    def print_log(self):
        wide_log("")
        with open(self.log_path, 'r') as f:
            for line in f:
                if not self.color:
                    wide_log(remove_ansi_escape(line.rstrip('\n')))
                else:
                    wide_log(line.rstrip('\n'))
        wide_log("")


class OutputController(object):
    def __init__(self, log_dir, quiet, interleave_output, color, prefix_output=False):
        self.log_dir = log_dir
        self.quiet = quiet
        self.interleave = interleave_output
        self.color = color
        self.prefix_output = prefix_output
        self.__command_log = {}

    def job_started(self, package):
        self.__command_log[package] = FileBackedLogCache(package, self.log_dir, self.color)
        wide_log("Starting ==> {package}".format(**locals()))

    def command_started(self, package, cmd, location):
        if package not in self.__command_log:
            raise RuntimeError("Command started received for package '{0}' before package job started: '{1}' in '{2}'"
                               .format(package, cmd, location))
        msg = "[{package}]: ==> '{cmd}' in '{location}'".format(**locals())
        self.__command_log[package].start_command(cmd, msg)
        if not self.quiet and self.interleave:
            wide_log(msg)

    def command_log(self, package, msg):
        if package not in self.__command_log:
            raise RuntimeError("Command log received for package '{0}' before package job started: '{1}'"
                               .format(package, msg))
        if self.__command_log[package].current_cmd is None:
            raise RuntimeError("Command log received for package '{0}' before command started: '{1}'"
                               .format(package, msg))
        self.__command_log[package].append(msg)
        if not self.quiet and self.interleave:
            if self.interleave and self.prefix_output:
                wide_log('[{package}]: {msg}'.format(**locals()))
            else:
                wide_log(msg)

    def command_failed(self, package, cmd, location, retcode):
        if package not in self.__command_log:
            raise RuntimeError(
                "Command failed received for package '{0}' before package job started: '{1}' in '{2}' returned '{3}'"
                .format(package, cmd, location, retcode))
        if self.__command_log[package].current_cmd is None:
            raise RuntimeError(
                "Command failed received for package '{0}' before command started: '{1}' in '{2}' returned '{3}'"
                .format(package, cmd, location, retcode))
        msg = "[{package}]: <== '{cmd}' in '{location}' failed with return code '{retcode}'".format(**locals())
        self.__command_log[package].finish_command(msg)
        self.__command_log[package].close()
        if not self.interleave:
            self.__command_log[package].print_log()
        del self.__command_log[package]

    def command_finished(self, package, cmd, location, retcode):
        if package not in self.__command_log:
            raise RuntimeError(
                "Command finished received for package '{0}' before package job started: '{1}' in '{2}' returned '{3}'"
                .format(package, cmd, location, retcode))
        if self.__command_log[package].current_cmd is None:
            raise RuntimeError(
                "Command finished received for package '{0}' before command started: '{1}' in '{2}' returned '{3}'"
                .format(package, cmd, location, retcode))
        msg = "[{package}]: <== '{cmd}' in '{location}' finished with return code '{retcode}'".format(**locals())
        self.__command_log[package].finish_command(msg)
        if not self.quiet and not self.interleave:
            self.__command_log[package].print_log()

    def job_finished(self, package, time):
        self.__command_log[package].close()
        if not self.quiet and not self.interleave:
            self.__command_log[package].print_log()
        del self.__command_log[package]
        wide_log("Finished <== {package} [ {time} ]".format(**locals()))
