"""Hosts files from the local directory using SSL."""
import ssl
import subprocess
from http.server import SimpleHTTPRequestHandler, HTTPServer

def main():
    """Main."""
    base_cert_file_name = 'www.pi-rc.com'
    try:
        with open(base_cert_file_name + '.cert'):
            pass
    except IOError:
        print(
'''Android requires HTTPS to access the webcam. This script can serve HTTPS
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
    server_address = ('0.0.0.0', 4443)
    httpd = HTTPServer(server_address, SimpleHTTPRequestHandler)
    httpd.socket = ssl.wrap_socket(
        httpd.socket,
        server_side=True,
        certfile='{}.cert'.format(base_cert_file_name),
        keyfile='{}.key'.format(base_cert_file_name),
        ssl_version=ssl.PROTOCOL_TLSv1
    )
    print('Running server')
    httpd.serve_forever()

if __name__ == '__main__':
    main()
