import os, re

def load_cmake_cache(bindir):
    ifile = open(os.path.join(bindir, 'CMakeCache.txt')).readlines()
    cache = {}
    for l in ifile:
        if l.startswith('//') or l.startswith('#') or len(l) < 2:
            continue

        m = re.match('(?P<key>[\w_\/\.\+-]+):(?P<type>\w+)=(?P<value>.*)$', l)
        if m:
            cache[m.group('key')] = m.group('value')
        else:
            raise RuntimeError("error parsing cmake cache line " + l)

    return cache

def prefixed(pfx, *args):
    return ', '.join([pfx + x for x in args])





