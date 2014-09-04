#!/usr/bin/env python
"""Manually send commands to the RC car."""
import argparse
import json
import pygame
import pygame.font
import socket
import sys

from common import server_up

UP = LEFT = DOWN = RIGHT = False
QUIT = False

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


def format_dune_warrior_command(throttle, turn, frequency):
    """Formats a command to JSON to the Raspberry Pi."""
    command = [format_command(frequency, 500)]
    if throttle >= 32 or throttle < 0:
        raise ValueError('Invalid throttle')
    # Turning too sharply causes the servo to push harder than it can go, so limit this
    if turn >= 58 or turn < 8:
        raise ValueError('Invalid turn')

    even_parity_bit = to_bit(
        (
            ones_count(throttle)
            + ones_count(turn)
            + 3
        ) % 2
    )

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
    if sys.version_info.major == 3:
        command_str = bytes(command_str, 'utf-8')
    return command_str


def get_keys():
    """Returns a tuple of (UP, DOWN, LEFT, RIGHT, changed) representing which
    keys are UP or DOWN and whether or not the key states changed.
    """
    change = False
    key_to_global_name = {
        pygame.K_LEFT: 'LEFT',
        pygame.K_RIGHT: 'RIGHT',
        pygame.K_UP: 'UP',
        pygame.K_DOWN: 'DOWN',
        pygame.K_ESCAPE: 'QUIT',
        pygame.K_q: 'QUIT',
    }
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            global QUIT
            QUIT = True
        elif event.type in {pygame.KEYDOWN, pygame.KEYUP}:
            down = (event.type == pygame.KEYDOWN)
            change = (event.key in key_to_global_name)

            if event.key in key_to_global_name:
                globals()[key_to_global_name[event.key]] = down

    return (UP, DOWN, LEFT, RIGHT, change)


def interactive_control(host, port, frequency):
    """Runs the interactive control."""
    pygame.init()
    size = (300, 400)
    screen = pygame.display.set_mode(size)
    # pylint: disable=too-many-function-args
    background = pygame.Surface(screen.get_size())
    clock = pygame.time.Clock()
    black = (0, 0, 0)
    white = (255, 255, 255)
    big_font = pygame.font.Font(None, 40)
    little_font = pygame.font.Font(None, 24)

    pygame.display.set_caption('Dune Warrior')

    text = big_font.render('Use arrows to move', 1, white)
    text_position = text.get_rect(centerx=size[0] / 2)
    background.blit(text, text_position)
    screen.blit(background, (0, 0))
    pygame.display.flip()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while not QUIT:
        up, down, left, right, change = get_keys()

        if change:
            # Something changed, so send a new command
            throttle = 16
            turn = 32
            if up:
                throttle = 24
            elif down:
                throttle = 8

            if left:
                turn = 12
            elif right:
                turn = 52

            command_json = format_dune_warrior_command(throttle, turn, frequency)
            sock.sendto(command_json, (host, port))

            # Show the command and JSON
            background.fill(black)
            text = big_font.render(command_json[:100], 1, white)
            text_position = text.get_rect(centerx=size[0] / 2)
            background.blit(text, text_position)

            pretty = json.dumps(json.loads(command_json), indent=4)
            pretty_y_position = big_font.size(command_json)[1] + 10
            for line in pretty.split('\n'):
                text = little_font.render(line, 1, white)
                text_position = text.get_rect(x=0, y=pretty_y_position)
                pretty_y_position += little_font.size(line)[1]
                background.blit(text, text_position)

            screen.blit(background, (0, 0))
            pygame.display.flip()

        # Limit to 20 frames per second
        clock.tick(60)

    pygame.quit()


def make_parser():
    """Builds and returns an argument parser."""
    parser = argparse.ArgumentParser(
        description='Interactive controller for the Raspberry Pi RC.'
    )
    parser.add_argument(
        '-p',
        '--port',
        dest='port',
        help='The port to send control commands to.',
        type=int,
        default=12345,
    )
    parser.add_argument(
        '-s',
        '--server',
        dest='server',
        help='The server to send control commands to.',
        type=str,
        default='127.1',
    )
    parser.add_argument(
        '-f',
        '--frequency',
        dest='frequency',
        help='The frequency to broadcast signals on.',
        type=float,
        default=27.145,
    )
    return parser


def main():
    """Parses command line arguments and runs the interactive controller."""
    parser = make_parser()
    args = parser.parse_args()

    print('Sending commands to ' + args.server + ':' + str(args.port))
    if not server_up(args.server, args.port, args.frequency):
        sys.stderr.write('Unable to contact server; did you start it?\n')
        sys.exit(1)

    interactive_control(args.server, args.port, args.frequency)

if __name__ == '__main__':
    main()
