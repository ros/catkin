# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
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

import sys
import os
import StringIO
import em

TEMPLATE_SUFFIX = '.em'


def instantiate_templates(templates_source_dir, globaldict, unittest=False):
    """
    crawls given dir for files ending with .em. Collect all
    files and folders, replacing template files with
    instantiation by empy
    :param templates_source_dir: where to look for templates
    :param globaldict: python environment for the templates to use as empy
    :returns: a dict {filepath: contents} with filepath relative to source dir
    """
    # first evaluate all templates, raise errors
    # if no error, then generate
    newfiles = {}
    for (path, dirs, files) in os.walk(templates_source_dir):
        target_dir_rel = os.path.relpath(path, templates_source_dir)
        if target_dir_rel == '.':
            target_dir_rel = ''
        for filename in files:
            content = None
            with open(os.path.join(path, filename)) as fhand:

                if not filename.endswith(TEMPLATE_SUFFIX):
                    realfilename = filename
                    if not filename.endswith('~'):
                        # not a template, copy the file
                        # we could also copy the file later
                        content = fhand.read()
                    else:
                        continue
                else:

                    realfilename = filename[:-len(TEMPLATE_SUFFIX)]
                    output = StringIO.StringIO()
                    # dirty hack here to make empy work with unit
                    # tests stdout wrapping
                    if unittest:
                        try:
                            sys.stdout._testProxy()
                        except:
                            sys.stdout = em.ProxyFile(sys.stdout)
                    interpreter = em.Interpreter(output=output)
                    for key, value in globaldict.items():
                        interpreter.__dict__[key] = value
                    try:
                        interpreter.file(fhand)
                        content = output.getvalue()
                    except Exception as nae:
                        print('Error in file: %s\n%s' %
                              (os.path.join(target_dir_rel, filename), str(nae)))
                        raise
                    finally:
                        interpreter.shutdown()
                if content is not None:
                    target_path = os.path.join(target_dir_rel, realfilename)
                    newfiles[target_path] = content

    return newfiles


def create_files(newfiles, target_dir):
    """
    writes file contents to target_dir/filepath for all entries of newfiles.
    Aborts early if files exist in places for new files or directories
    :param newfiles: a dict {filepath: contents}
    :param target_dir: a string
    """
    # first check no filename conflict exists
    for filename in newfiles:
        target_file = os.path.join(target_dir, filename)
        if os.path.exists(target_file):
            raise ValueError('File exists: %s' % target_file)
        dirname = os.path.dirname(target_file)
        while(dirname != target_dir):
            if os.path.isfile(dirname):
                raise ValueError('Cannot create directory, file exists: %s' %
                                 dirname)
            dirname = os.path.dirname(dirname)

    for filename, content in newfiles.items():
        target_file = os.path.join(target_dir, filename)
        dirname = os.path.dirname(target_file)
        if not os.path.exists(dirname):
            os.makedirs(dirname)
        # print(target_file, content)
        with open(target_file, 'ab') as fhand:
            fhand.write(content)
