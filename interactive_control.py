#!/usr/bin/env python
"""Interactive control for the remote radio control Raspberry Pi."""
import argparse
import json
import pygame
import pygame.font
import socket

from common import dead_frequency
from common import server_up


# pylint: disable=global-statement
# pylint: disable=invalid-name
# pylint: disable=superfluous-parens

UP = LEFT = DOWN = RIGHT = False
QUIT = False


def load_configuration(configuration_file):
    """Generates a dict of JSON command messages for each movement."""
    configuration = json.loads(configuration_file.read())
    dead = dead_frequency(configuration['frequency'])
    sync_command = {
        'frequency': configuration['frequency'],
        'dead_frequency': dead,
        'burst_us': configuration['synchronization_burst_us'],
        'spacing_us': configuration['synchronization_spacing_us'],
        'repeats': configuration['total_synchronizations'],
    }
    base_command = {
        'frequency': configuration['frequency'],
        'dead_frequency': dead,
        'burst_us': configuration['signal_burst_us'],
        'spacing_us': configuration['signal_spacing_us'],
    }

    movement_to_command = {}
    for key in (
        'forward',
        'forward_left',
        'forward_right',
        'left',
        'reverse',
        'reverse_left',
        'reverse_right',
        'right',
    ):
        command_dict = base_command.copy()
        command_dict['repeats'] = configuration[key]
        movement_to_command[key] = command_dict

    direct_commands = {
        key: json.dumps([sync_command, movement_to_command[key]])
        for key in movement_to_command
    }

    # We also need to add an idle command; just broadcast at the dead frequency
    command_dict = [base_command.copy()]
    command_dict[0]['frequency'] = dead
    command_dict[0]['repeats'] = 20  # Doesn't matter
    direct_commands['idle'] = json.dumps(command_dict)

    return direct_commands


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


def interactive_control(host, port, configuration):
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

    pygame.display.set_caption('rc-pi interactive')

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
            command = 'idle'
            if up:
                command = 'forward'
            elif down:
                command = 'reverse'

            append = lambda x: command + '_' + x if command != 'idle' else x

            if left:
                command = append('left')
            elif right:
                command = append('right')

            print(command)
            try:
                sock.sendto(configuration[command], (host, port))
            except TypeError:
                # Windows + Python 3 workaround?
                sock.sendto(bytes(configuration[command], 'utf-8'), (host, port))

            # Show the command and JSON
            background.fill(black)
            text = big_font.render(command, 1, white)
            text_position = text.get_rect(centerx=size[0] / 2)
            background.blit(text, text_position)

            pretty = json.dumps(json.loads(configuration[command]), indent=4)
            pretty_y_position = big_font.size(command)[1] + 10
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
        dest='control_file',
        help='JSON control file for the RC car.'
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
    return parser


def main():
    """Parses command line arguments and runs the interactive controller."""
    parser = make_parser()
    args = parser.parse_args()

    with open(args.control_file) as configuration_file:
        configuration = load_configuration(configuration_file)

    print('Sending commands to ' + args.server + ':' + str(args.port))

    frequency = json.loads(configuration['idle'])[0]['frequency']
    if not server_up(args.server, args.port, frequency):
        print('Server does not appear to be listening for messages, aborting')
        return

    interactive_control(args.server, args.port, configuration)

if __name__ == '__main__':
    main()
