#!/usr/bin/env python

from __future__ import print_function
import sys, subprocess, fileinput


def run(args):
  if len(args) < 2:
      raise Exception('no test file(s) specified')
  for f in args[1:]:
      for l in fileinput.input(f):
          ls = l.strip()
          if len(l) == 0 or l[0] == '#':
              continue
          cmd = ls.split()
          print('-- runtests.py: %s'%(ls))
          p = subprocess.Popen(cmd, 
                               stdout=subprocess.PIPE, 
                               stderr=subprocess.PIPE)
          stdout,stderr = p.communicate()
          sys.stdout.write(stdout)
          sys.stderr.write(stderr)

if __name__ == '__main__':
    try:
        run(sys.argv)
    except Exception as e:
        print("[runtests] Error: %s"%(e))
        sys.exit(-1)

