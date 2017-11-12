"""Hosts files from the local directory using SSL."""
from __future__ import print_function
import socket
import ssl
import subprocess
import sys

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
    decode = bytes(s, 'utf-8').decode('unicode-escape')


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
            post_data = decode(self.rfile.read(content_length)[1:-1])
            with open('parameters.json', 'w') as parameters_file:
                parameters_file.write(post_data)

            self.send_response(200)
            self.end_headers()

        else:
            self.send_response(404)
            self.end_headers()


def main():
    """Main."""
    # The URL fetching stuff inherits this timeout
    socket.setdefaulttimeout(0.25)
    # Prevent "address already in use" errors
    print(dir(Server))
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

    print('Starting server')
    port = 4443
    server_address = ('0.0.0.0', port)
    httpd = Server(server_address, PostCommandsRequestHandler)
    httpd.socket = ssl.wrap_socket(
        httpd.socket,
        server_side=True,
        certfile='{}.cert'.format(base_cert_file_name),
        keyfile='{}.key'.format(base_cert_file_name),
        ssl_version=ssl.PROTOCOL_TLSv1
    )
    print('Running server on https://localhost:4443/')
    httpd.serve_forever()


if __name__ == '__main__':
    main()
