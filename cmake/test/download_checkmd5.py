#!/usr/bin/env python

from __future__ import print_function
import os
import sys
import urllib
import hashlib
from optparse import OptionParser

NAME = "download_checkmd5.py"


def download_md5(uri, dest):
    """
    downloads file from uri to file dest
    """
    # Create intermediate directories as necessary, #2970
    dirname = os.path.dirname(dest)
    if len(dirname) and not os.path.exists(dirname):
        os.makedirs(dirname)

    sys.stdout.write('Downloading %s to %s...' % (uri, dest))
    sys.stdout.flush()
    urllib.urlretrieve(uri, dest)
    sys.stdout.write('Done\n')


def checkmd5(dest, md5sum=None):
    """
    checks file at dest against md5.
    :returns (boolean, hexdigest): True if dest contents matches md5sum
    """
    md5value = hashlib.md5(open(dest).read())
    hexdigest = md5value.hexdigest()

    print('Checking md5sum on %s' % (dest))
    return hexdigest == md5sum, hexdigest


def main(argv=sys.argv[1:]):
    """
    Dowloads URI to file dest and checks md5 if given.
    """
    parser = OptionParser(usage="usage: %prog URI dest [md5sum]",
                          prog=NAME,
                          description="Dowloads URI to file dest. If md5sum is given, checks md5sum. If file existed and mismatch, downloads and checks again")
    options, args = parser.parse_args(argv)
    md5sum = None
    if len(args) == 2:
        uri, dest = args
    elif len(args) == 3:
        uri, dest, md5sum = args
    else:
        parser.error("wrong number of arguments")
    fresh = False
    if not os.path.exists(dest):
        download_md5(uri, dest)
        fresh = True

    if md5sum:
        result, hexdigest = checkmd5(dest, md5sum)
        if result is False and fresh is False:
            print('WARNING: md5sum mismatch (%s != %s); re-downloading file %s' % (hexdigest, md5sum, dest))
            os.remove(dest)
            download_md5(uri, dest)
            result, hexdigest = checkmd5(dest, md5sum)
        if result is False:
            sys.exit('ERROR: md5sum mismatch (%s != %s) on %s;  aborting' % (hexdigest, md5sum, dest))


if __name__ == '__main__':
    main()
