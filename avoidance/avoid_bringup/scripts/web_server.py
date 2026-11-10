#!/usr/bin/env python3
"""Static server for the web UI (tmux window 'web'). Like `python3 -m http.server`, but tells the
browser not to cache: after a code update the page must not keep running the old app.js.

    web_server.py [port]      (serves ../web, all interfaces)
"""
import functools
import http.server
import os
import sys


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()


if __name__ == '__main__':
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    root = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'web')
    handler = functools.partial(NoCacheHandler, directory=os.path.realpath(root))
    with http.server.ThreadingHTTPServer(('', port), handler) as httpd:
        print('web UI on port %d, serving %s' % (port, os.path.realpath(root)), flush=True)
        httpd.serve_forever()
