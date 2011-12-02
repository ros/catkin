#!/usr/bin/env python

from __future__ import print_function
import os, sys, yaml, pprint, em, os.path, datetime, dateutil.tz, platform, catkin

cache = catkin.load_cmake_cache(sys.argv[5])

ctxfile = sys.argv[1]
ctx = {}
execfile(ctxfile, ctx)
cache.update(ctx)

infile = sys.argv[2]
templatedir = sys.argv[3]
outdir = sys.argv[4]

if not os.path.isdir(outdir):
    os.makedirs(outdir)

stackyaml = open(infile)


d = yaml.load(stackyaml)
d.update(ctx)
d.update(cache)
if 'Catkin-ChangelogType' not in d:
    d['Catkin-ChangelogType'] = ''

srcdir = d[d['Catkin-ProjectName'] + '_SOURCE_DIR']
def debexpand(name, d, filetype=''):
    ifilename = os.path.join(templatedir, name)

    if filetype != '':
        if filetype.startswith('+'):
            ifilename = os.path.join(srcdir, filetype[1:])
        else:
            ifilename += ('.' + filetype + '.em')
    else:
        ifilename += '.em'
    print("Reading %s template from %s" % (name, ifilename))
    file_em = open(ifilename).read()

    s = em.expand(file_em, **d)

    ofilename = outdir + '/' + name
    ofilestr = open(ofilename, "w")
    print(s, file=ofilestr)

t = datetime.datetime.now(dateutil.tz.tzlocal())
if 'CATKIN_DEB_SNAPSHOTS' in d:
    d['SnapshotVersion'] = '-' + t.strftime('%Y%m%d-%H%M%z')
else:
    d['SnapshotVersion'] = ''

d['Distribution'] = platform.dist()[2]
d['Date'] = t.strftime('%a, %d %b %Y %T %z')
d['YYYY'] = t.strftime('%Y')

debexpand('control', d)
debexpand('changelog', d, d['Catkin-ChangelogType'])
debexpand('rules', d, d['Catkin-DebRulesType'])
debexpand('copyright', d, d['Catkin-CopyrightType'])

sourcedir = os.path.join(outdir, "source")
if not os.path.isdir(sourcedir):
    os.makedirs(sourcedir)

fmtfile = open(os.path.join(sourcedir, "format"), "w")
print("3.0 (native)", file=fmtfile)
fmtfile.close()

compatfile = open(os.path.join(outdir, "compat"), "w")
print("7", file=compatfile)
compatfile.close()

