from __future__ import print_function
from sys import version_info as v

def main():
    """Prints python version to stdout"""
    # don't use version.major, version.minor, this is pys >= 2.7
    print("%u.%u" % (v[0], v[1]))


if __name__ == '__main__':
    main()
