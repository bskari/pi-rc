#!/usr/bin/env python
"""Manually send commands to the RC car."""
import argparse
import socket
import sys

from common import dead_frequency
from common import format_command
from common import server_up

# pylint: disable=superfluous-parens


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


def main():
    """Parses command line arguments and runs the simple controller."""
    parser = make_parser()
    args = parser.parse_args()

    if args.frequency is not None:
        frequency = dead_frequency(args.frequency)
    else:
        frequency = 49.830

    if not server_up(args.server, args.port, frequency):
        print('Server does not appear to be listening for messages, aborting')
        return

    command_array = get_command_array(parser)

    print('Sending commands to ' + args.server + ':' + str(args.port))
    send_signal_repeats(args.server, args.port, command_array)


if __name__ == '__main__':
    main()
