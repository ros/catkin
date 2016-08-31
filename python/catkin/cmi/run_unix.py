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
import pty
import select
import sys
import time

from subprocess import Popen
from subprocess import STDOUT


def process_incomming_lines(lines, left_over):
    if not lines:
        return None, left_over
    if lines[-1].endswith('\n'):
        data = b''.join(lines)
        left_over = b''
    else:
        data = b''.join(lines[:-1])
        left_over = lines[-1]
    return data, left_over


def run_command(cmd, cwd=None):
    master, slave = pty.openpty()

    p = None
    while p is None:
        try:
            p = Popen(cmd, stdin=slave, stdout=slave, stderr=STDOUT, cwd=cwd)
        except OSError as exc:
            if 'Text file busy' in str(exc):
                # This is a transient error, try again shortly
                time.sleep(0.01)
                continue
            raise
    if sys.platform.startswith('darwin'):
        os.close(slave)  # This causes the below select to exit when the subprocess closes

    left_over = b''

    # Read data until the process is finished
    while p.poll() is None:
        incomming = left_over
        rlist, wlist, xlist = select.select([master], [], [], 0.1)
        if rlist:
            incomming += os.read(master, 1024)
            lines = incomming.splitlines(True)  # keepends=True
            data, left_over = process_incomming_lines(lines, left_over)
            if data is None:
                continue
            yield data

    # Done
    os.close(master)
    yield p.returncode
