#!/bin/env python
"""Hosts files from the local directory using SSL."""
from __future__ import print_function
import signal
import socket
import ssl
import subprocess
import sys
import threading


killed = False


# pylint: disable=C0411
if sys.version_info.major < 3:
    import SimpleHTTPServer
    import SocketServer
    import urllib
    Server = SocketServer.TCPServer
    SimpleHTTPRequestHandler = SimpleHTTPServer.SimpleHTTPRequestHandler
    urlopen = urllib.urlopen
    decode = lambda s: s.decode('string_escape')
else:
    from http.server import SimpleHTTPRequestHandler, HTTPServer  # pylint: disable=E0401
    Server = HTTPServer  # pylint: disable=C0103
    import urllib.request
    urlopen = urllib.request.urlopen
    decode = lambda s: bytes(s, 'utf-8').decode('unicode-escape')


class InterruptibleServer(Server):
    def __init__(self, server_address, handler):
        if sys.version_info.major < 3:
            # Python 2's TCPServer is an old style class
            Server.__init__(self, server_address, handler)
        else:
            super().__init__(server_address, handler)

    def serve_until_shutdown(self):
        global killed
        while not killed:
            self.handle_request()


class PostCommandsRequestHandler(SimpleHTTPRequestHandler):  # pylint: disable=R0903
    """Serves files over GET and handles commands send over POST."""

    def do_POST(self):  # pylint: disable=C0103
        """Handles POST requests."""
        if not self.path.endswith('/'):
            # Redirect browser - doing basically what Apache does
            self.send_response(301)
            self.send_header('Location', self.path + '/')
            self.end_headers()

        elif self.path == '/command/':
            # Forward this request on to the C server, because doing SSL in C
            # sounds hard
            content_length = int(self.headers.get('Content-Length'))
            post_data = self.rfile.read(content_length)
            print(post_data)

            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect(('localhost', 12345))
                sock.sendall(post_data)
            except Exception as exc:
                print('{}, sending 500'.format(exc))
                self.send_response(500)
                self.send_header('Content-type', 'text/plain; charset=utf-8')
                self.end_headers()
                # Firefox keeps expecting to get XML back. If we send back
                # plain text, it doesn't error out, but it generates a console
                # warning, so let's just play nice
                self.wfile.write('<p>Unable to contact pi_pcm; is it running?</p>')
                return
            finally:
                sock.close()

            self.send_response(200)
            self.end_headers()

        elif self.path == '/save/':
            content_length = int(self.headers.get('Content-Length'))
            post_data = decode(self.rfile.read(content_length))
            with open('parameters.json', 'w') as parameters_file:
                parameters_file.write(post_data)

            self.send_response(200)
            self.end_headers()

        else:
            self.send_response(404)
            self.end_headers()


def kill_servers(*_):
    global killed
    killed = True


def main():
    """Main."""
    signal.signal(signal.SIGINT, kill_servers)
    # The URL fetching stuff inherits this timeout
    socket.setdefaulttimeout(0.25)
    # Prevent "address already in use" errors
    Server.allow_reuse_address = True

    base_cert_file_name = 'www.pi-rc.com'
    try:
        with open(base_cert_file_name + '.cert'):
            pass
    except IOError:
        print(
'''Chrome requires HTTPS to access the webcam. This script can serve HTTPS
requests, but requires that a self-signed certificate be generated first. When
you access this page, you will get a warning - just click through it. This
script will now generate a self-signed certificate.'''
        )
        subprocess.call((
            'openssl',
            'req',
            '-new',
            '-newkey',
            'rsa:4096',
            '-days',
            '365',
            '-nodes',
            '-x509',
            '-subj',
            '/C=US/ST=Denial/L=Springfield/O=Dis/CN={}'.format(base_cert_file_name),
            '-keyout',
            '{}.key'.format(base_cert_file_name),
            '-out',
            '{}.cert'.format(base_cert_file_name)
        ))

    print('Starting servers')

    secure_port = 4443
    server_address = ('0.0.0.0', secure_port)
    secure_httpd = InterruptibleServer(server_address, PostCommandsRequestHandler)
    secure_httpd.socket = ssl.wrap_socket(
        secure_httpd.socket,
        server_side=True,
        certfile='{}.cert'.format(base_cert_file_name),
        keyfile='{}.key'.format(base_cert_file_name),
        ssl_version=ssl.PROTOCOL_TLSv1
    )

    insecure_port = 8080
    server_address = ('0.0.0.0', insecure_port)
    insecure_httpd = InterruptibleServer(server_address, PostCommandsRequestHandler)

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        skari_org = '149.154.158.78'
        # This won't actually make a connection
        sock.connect((skari_org, 1))
        ip = sock.getsockname()[0]
    except socket.gaierror:
        ip = 'localhost'
    finally:
        sock.close()
    print(
            'Running server on https://{ip}:{secure_port}/ and http://{ip}:{insecure_port}/'.format(
                ip=ip,
                secure_port=secure_port,
                insecure_port=insecure_port
        )
    )
    secure_thread = threading.Thread(target=lambda: secure_httpd.serve_until_shutdown())
    secure_thread.start()
    insecure_httpd.serve_until_shutdown()


if __name__ == '__main__':
    main()
