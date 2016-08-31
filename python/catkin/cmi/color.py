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

"""This module implements many of the colorization functions used by cmi"""

from catkin.cmi.terminal_color import enable_ANSI_colors
from catkin.cmi.terminal_color import disable_ANSI_colors
from catkin.cmi.terminal_color import fmt
from catkin.cmi.terminal_color import sanitize

# This map translates more human reable format strings into colorized versions
_color_translation_map = {
    # 'output': 'colorized_output'
    '': fmt('@!' + sanitize('') + '@|'),

    "[{package}]: ==> '{cmd.cmd_str}' in '{location}'":
    fmt("[@{cf}{package}@|]: @!@{bf}==>@| '@!{cmd.cmd_str}@|' @{kf}@!in@| '@!{location}@|'"),

    "Starting ==> {package}":
    fmt("Starting @!@{gf}==>@| @!@{cf}{package}@|"),

    "[{package}]: {msg}":
    fmt("[@{cf}{package}@|]: {msg}"),

    "[{package}]: <== '{cmd.cmd_str}' failed with return code '{retcode}'":
    fmt("[@{cf}{package}@|]: @!@{rf}<==@| '@!{cmd.cmd_str}@|' @{rf}failed with return code@| '@!{retcode}@|'"),

    "[{package}]: <== '{cmd.cmd_str}' finished with return code '{retcode}'":
    fmt("[@{cf}{package}@|]: @{gf}<==@| '@!{cmd.cmd_str}@|' finished with return code '@!{retcode}@|'"),

    "Finished <== {package} [ {time} ]":
    fmt("@!@{kf}Finished@| @{gf}<==@| @{cf}{package}@| [ @{yf}{time}@| ]"),

    "[{name} - {run_time}] ":
    fmt("[@{cf}{name}@| - @{yf}{run_time}@|] "),

    "[{0}/{1} Active | {2}/{3} Completed]":
    fmt("[@!@{gf}{0}@|/@{gf}{1}@| Active | @!@{gf}{2}@|/@{gf}{3}@| Completed]"),

    "[!{package}] ":
    fmt("[@!@{rf}!@|@{cf}{package}@|] "),
}


def colorize_cmake(line):
    """Colorizes output from CMake

    :param line: one, new line terminated, line from `cmake` which needs coloring.
    :type line: str
    """
    cline = sanitize(line)
    if line.startswith('-- '):
        cline = '@{cf}-- @|' + cline[len('-- '):]
        if ':' in cline:
            split_cline = cline.split(':')
            cline = split_cline[0] + ':@{yf}' + ':'.join(split_cline[1:]) + '@|'
    if line.lower().startswith('warning'):
        # WARNING
        cline = fmt('@{yf}') + cline
    if line.startswith('CMake Warning'):
        # CMake Warning...
        cline = cline.replace('CMake Warning', '@{yf}@!CMake Warning@|')
    if line.startswith('ERROR:'):
        # ERROR:
        cline = cline.replace('ERROR:', '@!@{rf}ERROR:@|')
    if line.startswith('CMake Error'):
        # CMake Error...
        cline = cline.replace('CMake Error', '@{rf}@!CMake Error@|')
    if line.startswith('Call Stack (most recent call first):'):
        # CMake Call Stack
        cline = cline.replace('Call Stack (most recent call first):',
                              '@{cf}@_Call Stack (most recent call first):@|')
    return fmt(cline)

_color_on = True


def set_color(state):
    """Sets the global colorization setting.

    Setting this to False will cause all ansi colorization sequences to get
    replaced with empty strings.

    :parma state: colorization On or Off, True or False respectively
    :type state: bool
    """
    global _color_on
    if state:
        enable_ANSI_colors()
        _color_on = True
    else:
        disable_ANSI_colors()
        _color_on = False


def clr(key):
    """Returns a colorized version of the string given.

    This is occomplished by either returning a hit from the color translation
    map or by calling :py:func:`fmt` on the string and returning it.

    :param key: string to be colorized
    :type key: str
    """
    global _color_translation_map, _color_on
    if not _color_on:
        return fmt(key)
    val = _color_translation_map.get(key, None)
    if val is None:
        return fmt(key)
    return val
