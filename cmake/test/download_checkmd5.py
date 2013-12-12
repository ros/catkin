#!/usr/bin/env python

from __future__ import print_function
import os
import sys
import urllib2
import time
import hashlib
from optparse import OptionParser

NAME = "download_checkmd5.py"


# Support retry on dropped connection: #559
class HTTPRangeHandler(urllib2.BaseHandler):
    """
    handler that enables HTTP Range headers.
    """
    # Taken from urlgrabber: http://urlgrabber.baseurl.org

    def http_error_206(self, req, fp, code, msg, hdrs):
        # 206 Partial Content Response
        r = urllib.addinfourl(fp, hdrs, req.get_full_url())
        r.code = code
        r.msg = msg
        return r

    def http_error_416(self, req, fp, code, msg, hdrs):
        # HTTP's Range Not Satisfiable error
        raise RangeError('Requested Range Not Satisfiable')
      

def urlretrieve(url, localfile=None, nRetry=20, append=False, progressFn=None):
    """ 
    downloads the file pointed to by the url. Supports retrying and resuming
    a partially downloaded file.

    Arguments:
    ----------
    
    url: string or anything supported by urllib
        The URL of the file to download.
    localfile: string (defaults to None)
        The name of the resulting file. If None, the basename of the url
        is used.
    nRetry: integer (defaults to 20)
        The max number of trials.
    append: boolean (defaults to False)
        In case the localfile exists, whether we want to append to it,
        i.e. resume a partial download. If false, then the localfile is
        first deleted.
    progressFn: function (defaults to None)
        If not None, the function is called when new data is downloaded,
        with 2 arguments: the number of bytes downloaded so far and
        the total number of bytes.
    """

    # TODO:
    #   - there is probably a number of errors that can occur when reading
    #     where we would want to retry. At the moment we just fail.
    #   - If the range header is not supported, an exception is thrown.
    #     There might be a better strategy. For instance: delete the localfile
    #     and attempt to download it again from scratch ...

    if localfile is None:
        localfile = url.split('/')[-1]
    
    # If the file exists but we do not wish to append to it then erase it.
    if not append and os.path.exists(localfile) and os.path.isfile(localfile):
        os.remove(localfile)

    range_handler = HTTPRangeHandler()
    opener = urllib2.build_opener(range_handler)

    for retry in range(nRetry):
        _netfile = opener.open(url)
        filesize = float(_netfile.info()['Content-Length'])

        count = 0
        if os.path.exists(localfile) and os.path.isfile(localfile):
            count = os.path.getsize(localfile)
            if count >= filesize:
                # we are done
                _netfile.close()
                return
                
            #print("resuming: %d of %d downloaded." % (count, filesize))
            _netfile.close()
            req = urllib2.Request(url)
            req.add_header("Range","bytes=%s-" % (count))
            _netfile = opener.open(req)

        _outfile = open(localfile, "ab")
        data = None
        chunk_size = 8 * 1024
        while data is None or len(data)>0:
            data = _netfile.read(chunk_size)
            _outfile.write(data)
            count += len(data)

            if progressFn:
                progressFn(count, filesize)

            if count >= filesize:
                # we are done
                _netfile.close()
                _outfile.close()
                return

        # we reach this point if the read returned no bytes. In which case we
        # want to retry
        _netfile.close()
        _outfile.close()

        # is it necessary ???
        time.sleep(1)
        
    raise IOError('Failed to download file. Exceeded number of trials.')


def progress_report(bytes_so_far, total_size):
    """ 
    progress report function that can be used with urlretrieve().
    """
    percent = float(bytes_so_far) / total_size
    percent = round(percent*100, 2)
    sys.stdout.write("Downloaded %d of %d bytes (%0.2f%%)\r" % (bytes_so_far, total_size, percent))
    if bytes_so_far >= total_size:
        sys.stdout.write('\n')


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
    urlretrieve(uri, dest)
    sys.stdout.write('Done\n')


def checkmd5(dest, md5sum=None):
    """
    checks file at dest against md5.
    :returns (boolean, hexdigest): True if dest contents matches md5sum
    """
    with open(dest, 'rb') as f:
        md5value = hashlib.md5()
        while True:
            buf = f.read(4096)
            if not buf:
                break
            md5value.update(buf)
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
