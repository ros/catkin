import sys
import os
import re
from optparse import OptionParser

"""Simple superficial API doc generator for .cmake files"""

def generate(path, filename):
    """
    generate crawls over path, looking for files named *.cmake,
    indexing functions and macros with docstings, and writing the
    result to filename.

    Each of the crawled files is traversed line by line, looking for
    lines like function(...) or macro(...)  for each of these, an
    entry is added to the overall result in reStructured text syntax
    for Sphinx documentation
    """
    # print(path, filename)
    macro_files = []
    rst_txt = ['Automated macro reference\n',
               '=========================\n\n']

    for (parentdir, _, files) in os.walk(path):
        # if os.path.basename(parentdir) != 'cmake':
        #     continue
        for filename2 in files:
            if not filename2.endswith('.cmake'):
                continue
            fullpath = (os.path.join(parentdir, filename2))
            relpath = os.path.relpath(fullpath, path)
            macro_files.append((fullpath, relpath))

    for (macro_file, relpath) in macro_files:
        lastblock = []
        with open(macro_file, 'r') as fhand:
            contents = fhand.readlines()
        for line in contents:
            if line.startswith('#'):
                # careful to not strip '\n' in empty comment lines
                lastblock.append(line.lstrip('#'))
            else:
                declaration = re.match("[a-zA-Z]+\([a-zA-Z_ ]+\)", line)
                if declaration is None:
                    lastblock = []
                else:
                    tokens = line.split('(')
                    dec_type = tokens[0].strip()
                    dec_args = tokens[1].strip().rstrip(')').split(' ')
                    
                    
                    if dec_type == 'function':
                        # directives defined in catkin-sphinx
                        dec_line = '.. cmake:macro:: '+dec_args[0]
                    elif dec_type == 'macro':
                        dec_line = '.. cmake:macro:: '+dec_args[0]
                    else:
                        dec_line = None

                    if dec_line is None:
                        lastblock = []
                    else:
                        # print(line, dec_type, dec_args)
                        dec_line += '(%s)\n\n'%', '.join(dec_args[1:])
                        rst_txt.append(dec_line)
                        rst_txt.append('   defined in %s\n\n'%relpath)
                        
                        if lastblock != []:
                            rst_txt.extend(lastblock)
                            lastblock = []
                        else:
                            rst_txt.append("   No docstring\n")
                        rst_txt.append('\n\n')

    with open(filename, 'w') as fhand:
        fhand.write(''.join(rst_txt))

    return 0

if __name__ == "__main__":
    parser = OptionParser(usage="%s path [path] [options]"%sys.argv[0])
    parser.add_option("-o", "--ouput", dest="output", default="macros.rst",
                      help="output filename",
                      action="store")
    options, args = parser.parse_args()
    if len(args) > 1:
        parser.usage()
        sys.exit(1)
    if len(args) > 0:
        path = args[0]
    else:
        path = os.path.dirname(os.getcwd())

    sys.exit(generate(path, options.output))

