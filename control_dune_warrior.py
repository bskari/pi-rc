"""Manually send commands to the RC car."""
import argparse
import json
import socket
import sys

from common import server_up

# pylint: disable=superfluous-parens

def dead_frequency(frequency):
    """Returns an approprtiate dead signal frequency for the given signal."""
    if frequency < 38:
        return 49.890
    return 26.995


def format_command(
    frequency,
    useconds
):
    """Returns the JSON command string for this command tuple."""
    dead = dead_frequency(frequency)
    return {
        'frequency': frequency,
        'dead_frequency': dead,
        'burst_us': useconds,
        'spacing_us': useconds,
        'repeats': 1,
    }


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
    }

    for option, prompt_and_function in option_to_prompt_and_function.items():
        if getattr(args, option) is None:
            prompt, function = prompt_and_function
            setattr(args, option, function(prompt))

    return [
        float(args.frequency),
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

    return parser


def to_bit(number):
    if number > 0:
        return 1
    return 0


def ones_count(number):
    mask = 1
    ones = 0
    while mask <= number:
        ones += to_bit(mask & number)
        mask <<= 1
    return ones


def send_signal(host, port, frequency):
    """Reads signal repeat bursts and sends commands to the Raspberry Pi."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    read_int = input_function(int)

    while True:
        command = [format_command(frequency, 500)]
        throttle = read_int('Throttle? (16 is idle)' )
        turn = read_int('Turn? (32 is center, 0 is left)' )
        if throttle >= 32 or throttle < 0:
            print('Invalid throttle')
            continue
        # Turning too sharply causes the servo to push harder than it can go, so limit this
        if turn >= 58 or turn < 8:
            print('Invalid turn')
            continue

        print(ones_count(throttle))
        print(ones_count(turn))
        even_parity_bit = to_bit(
            (
                ones_count(throttle)
                + ones_count(turn)
                + 3
            ) % 2
        )
        print(even_parity_bit)

        bit_pattern = (
            to_bit(turn & 0x8),
            to_bit(turn & 0x4),
            to_bit(turn & 0x2),
            to_bit(turn & 0x1),
            0,
            0,
            to_bit(turn & 0x20),
            to_bit(turn & 0x10),
            to_bit(throttle & 0x10),
            to_bit(throttle & 0x8),
            to_bit(throttle & 0x4),
            to_bit(throttle & 0x2),
            to_bit(throttle & 0x1),
            1,
            1,
            1,
            0,
            0,
            even_parity_bit,
            0,
            0,
            0
        )
        assert(len(bit_pattern) == 22)
        print(bit_pattern)
        print(sum(bit_pattern))
        assert(sum(bit_pattern) % 2 == 0)

        total_useconds = 1000
        for bit in bit_pattern[:-1]:
            if bit == 0:
                useconds = 127
            else:
                useconds = 200
            command.append(format_command(27.145, useconds))
            total_useconds += useconds

        if bit_pattern[-1] == 0:
            useconds = 127
        else:
            useconds = 200
        total_useconds += useconds
        command.append({
            'frequency': frequency,
            'dead_frequency': dead_frequency(frequency),
            'burst_us': useconds,
            'spacing_us': 7000 - total_useconds,
            'repeats': 1,
        })

        command_str = json.dumps(command)
        print(command_str)
        if sys.version_info.major == 3:
            command_str = bytes(command_str, 'utf-8')
        sock.sendto(command_str, (host, port))

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
    send_signal(args.server, args.port, command_array[0])


if __name__ == '__main__':
    main()
