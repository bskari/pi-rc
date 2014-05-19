"""Common functions for the Raspberry Pi radio controller."""
import json
import socket
import sys


def dead_frequency(frequency):
    """Returns an approprtiate dead signal frequency for the given signal."""
    if frequency < 38:
        return 49.890
    return 26.995


def format_command(
    frequency,
    useconds,
    sync_multiplier,
    sync_repeats,
    signal_repeats
):
    """Returns the JSON command string for this command tuple."""
    dead = dead_frequency(frequency)
    return json.dumps([
        {
            'frequency': frequency,
            'dead_frequency': dead,
            'burst_us': useconds * sync_multiplier,
            'spacing_us': useconds,
            'repeats': sync_repeats,
        },
        {
            'frequency': frequency,
            'dead_frequency': dead,
            'burst_us': useconds,
            'spacing_us': useconds,
            'repeats': signal_repeats,
        }
    ])


def server_up(host, port, frequency):
    """Checks that the server is up and listening to commands."""
    # Send a test command to make sure that the server is listening
    listen_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listen_socket.bind(('', port + 1))
    listen_socket.settimeout(1.0)
    send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    command = json.dumps([{
        'frequency': dead_frequency(frequency),
        'dead_frequency': dead_frequency(frequency),
        'burst_us': 100,
        'spacing_us': 100,
        'repeats': 10,
        'request_response': True,  # This forces the server to respond
    }])
    response_received = False
    for _ in range(3):
        if sys.version_info.major == 3 and isinstance(command, str):
            command = bytes(command, 'utf-8')
        send_socket.sendto(command, (host, port))
        try:
            listen_socket.recv(1024)
            response_received = True
            break
        except socket.timeout:
            pass

    return response_received
