import re

def load_cmake_cache(bindir):
    ifile = open(os.path.join(bindir, 'CMakeCache.txt')).readlines()
    for l in ifile:
        if l.startswith('//') or l.startswith('\#') or len(l) < 2:
            print "skip", l
            continue





