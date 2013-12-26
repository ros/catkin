#!/usr/bin/env python

from __future__ import print_function
import os
import sys
try:
    from urllib.request import urlopen, BaseHandler, build_opener, Request, addinfourl, urlretrieve
except ImportError:
    from urllib2 import urlopen, BaseHandler, build_opener, Request, addinfourl
    from urllib import urlretrieve
import time
import hashlib
from optparse import OptionParser

NAME = "download_checkmd5.py"


# Support retry on dropped connection: #559
class HTTPRangeHandler(BaseHandler):
    """
    handler that enables HTTP Range headers.
    """
    # Taken from urlgrabber: http://urlgrabber.baseurl.org

    def http_error_206(self, req, fp, code, msg, hdrs):
        # 206 Partial Content Response
        r = addinfourl(fp, hdrs, req.get_full_url())
        r.code = code
        r.msg = msg
        return r

    def http_error_416(self, req, fp, code, msg, hdrs):
        # HTTP's Range Not Satisfiable error
        raise RangeError('Requested Range Not Satisfiable')


class FileDownloader:
    """
    File downloader.

    Parameters:
    -----------
    - nRetry: integer (defaults to 3)
      The max number of consecutive failures to download some bytes from the url.
    - append: boolean (defaults to False)
      In case the localfile exists, whether we want to append to it,
      i.e. resume a partial download. If false, then the localfile is
      first deleted.
    - progressFn: function (defaults to None)
      If not None, the function is called when new data is downloaded,
      with 2 arguments: the number of bytes downloaded so far and
      the total number of bytes.
    """
    
    def __init__(self):
        """
        Constructor: provides default value for the parameters.
        """
        self.nRetry = 3
        self.append = False
        self.progressFn = None
        self.chunk_size = 8 * 1024
        self.verbose = False


    def download_file(self, url, localfile=None):
        """
        downloads the file pointed to by the url. Supports retrying and resuming
        a partially downloaded file.

        Arguments:
        ----------
        - url: string or anything supported by urllib
          The URL of the file to download.
        - localfile: string (defaults to None)
          The name of the resulting file. If None, the basename of the url
          is used.
        """

        self.count = 0
        self.lastProgressReportPercent = None
        self.retry = 0

        self.url = url
        # The 'file://' prefix is mandatory with the our HTTPRangeHandler
        # (used to be optional with urllib.urlretrieve).
        # Restoring for backward compatibility.
        if '://' not in self.url:
            self.url = 'file://' + self.url

        self.localfile = localfile
        if self.localfile is None:
            self.localfile = os.path.basename(self.url)

        # If the file exists but we do not wish to append to it then erase it.
        # If we want to append to it, then get its size.
        if os.path.exists(self.localfile) and os.path.isfile(self.localfile):
            if self.append:
                self.count = os.path.getsize(self.localfile)
            else:
                os.remove(self.localfile)

        self.range_handler = HTTPRangeHandler()
        self.opener = build_opener(self.range_handler)

        # Check if the server provides the remote file size and if it supports
        # range header. If it does not, then fallback to using urlretrieve
        # (and download the whole file without being able to recover from errors)
        netfile = self._connect(self.url)
        header = netfile.info()
        netfile.close()
        if not header.has_key('Content-Length') or not header.has_key('Accept-Ranges') or header['Accept-Ranges']=='none':
            urlretrieve(self.url, self.localfile)
            return
            
        self.filesize = int(header['Content-Length'])
        if self.count == self.filesize:
            return
        elif self.count > self.filesize:
            raise IOError('Resulting file is larger than source file')

        while not self._download(): pass

                
    def _connect(self, req):
        """
        Tries to open a connection to the server (with retrial).
        Returns the file-like object returned by urlopen on success. Raises
        an IOError if the number of retrials has been exceeded.
        """
        while True:
            try:
                return self.opener.open(req)
            except:
                # As wget does, we wait 1 second after the 1st failure, 2 seconds
                # after the 2nd, etc.
                self.retry = self.retry + 1
                if self.retry < self.nRetry:
                    time.sleep(retry)
                else:
                    raise IOError('Failed to download file. Exceeded number of trials.')


    def _progress(self):
        if self.progressFn is not None:
            percent = float(self.count)/self.filesize*100
            if self.lastProgressReportPercent is None or percent >= self.lastProgressReportPercent+1:
                self.progressFn(self.count, self.filesize)
                self.lastProgressReportPercent = int(float(self.count)/self.filesize*100)


    def _check_content_range(self, header):
        """
        Checks that the response from the server contains a valid Content-Range
        field. Raises an IOError if it does not.
        """
        msg = None
        if not header.has_key('Content-Range'):
            msg = 'server did not reply with Content-Range'
        else:
            try:
                if int(header['Content-Range'][len('bytes '):].split('-')[0]) != self.count:
                    msg = 'server replied with a Content-Range starting at the wrong place'
            except:
                msg = 'failed to parse the Content-Range header'
        if msg:
            raise IOError('Failed to resume the download: %s.' % msg)

    
    def _download(self):
        """
        Downloads the next bytes from the file. Returns True if the download
        completed, False otherwise in which case we want to reopen the connection
        and try again.
        Raises IOErrors if something goes wrong.
        """
        if self.count>0 and self.verbose:
            print("\n\nresuming: %d of %d downloaded.\n" % (self.count, self.filesize))

        netfile = None
        outfile = None
        try:
            req = Request(self.url)
            req.add_header("Range","bytes=%d-" % (self.count))
            netfile = self._connect(req)
            self._check_content_range(netfile.info())
            outfile = open(self.localfile, "ab")
            data = None
            while data is None or len(data)>0:
                data = netfile.read(self.chunk_size)
                if len(data)>0: self.retry = 0
                outfile.write(data)
                self.count += len(data)
                self._progress()
                if self.count == self.filesize:
                    return True
                elif self.count > self.filesize:
                    raise IOError('Resulting file is larger than source file')
        finally:
            if netfile: netfile.close()
            if outfile: outfile.close()
        return False


def progress_report(bytes_so_far, total_size):
    """
    progress report function that can be used with download_file().
    """
    percent = float(bytes_so_far) / total_size
    percent = round(percent*100, 2)
    sys.stdout.write("Downloaded %d of %d bytes (%0.2f%%)\r" % (bytes_so_far, total_size, percent))
    sys.stdout.flush()
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

    downloader = FileDownloader()
    downloader.progressFn = progress_report
    downloader.verbose = True
    downloader.download_file(uri, dest)
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
            sys.exit('ERROR: md5sum mismatch (%s != %s) on %s; aborting' % (hexdigest, md5sum, dest))


if __name__ == '__main__':
    main()
