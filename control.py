"""Manually send commands to the RC car."""
import argparse
import json
import socket
import sys

# pylint: disable=superfluous-parens


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


def input_function(type_cast):
    """Returns the input function for the running version of Python for reading
    data from stdin.
    """
    # pylint: disable=bad-builtin
    if sys.version_info.major == 2:
        return lambda message: type_cast(raw_input(message))
    else:
        return lambda message: type_cast(input(message))


def get_command_array(parser):
    """Returns an array of command information that can be used in the
    format_command function.
    """
    args = parser.parse_args()

    read_float = input_function(float)
    read_int = input_function(int)

    option_to_prompt_and_function = {
        'frequency': ('Command frequency? ', read_float),
        'microseconds': ('Microseconds? ', read_int),
        'sync_multiplier': ('Synchronization multiplier? ', read_int),
        'sync_repeats': ('Synchronization repeats? ', read_int),
    }

    for option, prompt_and_function in option_to_prompt_and_function.items():
        if getattr(args, option) is None:
            prompt, function = prompt_and_function
            setattr(args, option, function(prompt))

    return [
        float(args.frequency),
        int(args.microseconds),
        int(args.sync_multiplier),
        int(args.sync_repeats),
        0,  # Signal repeats, to be read in and configured later
    ]


def make_parser():
    """Builds and returns an argument parser."""
    parser = argparse.ArgumentParser(
        description='Sends burst commands to Raspberry Pi RC.'
    )

    parser.add_argument(
        '-p',
        '--port',
        dest='port',
        help='The port to send control commands to.',
        default=12345,
        type=int
    )
    parser.add_argument(
        '-s',
        '--server',
        dest='server',
        help='The server to send control commands to.',
        default='127.1'
    )

    parser.add_argument(
        '-f',
        '--frequency',
        dest='frequency',
        help='The frequency to broadcast commands on.'
    )
    parser.add_argument(
        '-u',
        '--microseconds',
        dest='microseconds',
        help='The interval in microseconds for the commands.'
    )
    parser.add_argument(
        '--sync-multiplier',
        dest='sync_multiplier',
        help='The multiplier of the interval microseconds for the'
            ' synchronization burst. For example, if the microseconds is 100'
            ' and the multiplier is 3, each synchronization burst will have'
            ' 300 us of signal and 100 us of dead signal.'
    )
    parser.add_argument(
        '--sync-repeats',
        dest='sync_repeats',
        help='The number of times to repeat the synchronization bursts.'
    )

    return parser


def send_signal_repeats(host, port, command_array):
    """Reads signal repeat bursts and sends commands to the Raspberry Pi."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    read_int = input_function(int)

    while True:
        try:
            command_array[4] = read_int('Signal repeats? ')
        except ValueError:
            pass
        # pylint: disable=star-args
        command = format_command(*command_array)
        if sys.version_info.major == 3:
            command = bytes(command, 'utf-8')
        sock.sendto(command, (host, port))


def dead_frequency(frequency):
    """Returns an approprtiate dead signal frequency for the given signal."""
    if frequency < 38:
        return 49.890
    return 26.995


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
        send_socket.sendto(command, (host, port))
        try:
            listen_socket.recv(1024)
            response_received = True
            break
        except socket.timeout:
            pass

    return response_received


def main():
    """Parses command line arguments and runs the simple controller."""
    parser = make_parser()
    args = parser.parse_args()

    if not server_up(args.server, args.port, dead_frequency(args.frequency)):
        print('Server does not appear to be listening for messages, aborting')
        return

    command_array = get_command_array(parser)

    print('Sending commands to ' + args.server + ':' + str(args.port))
    send_signal_repeats(args.server, args.port, command_array)


if __name__ == '__main__':
    main()
