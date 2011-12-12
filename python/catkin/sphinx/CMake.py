#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

from docutils.parsers import rst
import docutils.nodes

class cmake(docutils.nodes.Element): pass

class CMakeDirective(rst.Directive):
    has_content = True
    final_argument_whitespace = True
    required_arguments = 1

    option_spec = dict(flag=rst.directives.flag)

    def run(self):
        node = cmake()
        contentnode = docutils.nodes.paragraph()
        self.state.nested_parse(self.content, self.content_offset, contentnode)
        node.content = contentnode
        return [node]

def do_cmake(app, doctree):
    #print doctree
    for node in doctree.traverse(cmake):
        print (("v"*50) +"\n")*3
        print node.content
        print (("^"*50) +"\n")*3
        para = docutils.nodes.title("EEEP")
        para += docutils.nodes.paragraph(text=node.content)
        node.replace_self(para)

def setup(app):
    app.add_directive('cmake', CMakeDirective)
    app.connect('doctree-read', do_cmake)
