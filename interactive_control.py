"""Interactive control for the remote radio control Raspberry Pi."""
import json
import pygame
import pygame.font
import socket
import sys

UP = LEFT = DOWN = RIGHT = False
QUIT = False


def load_configuration(configuration_file):
    """Generates a dict of JSON command messages for each movement."""
    configuration = json.loads(configuration_file.read())
    dead_frequency = 49.890 if configuration['frequency'] < 38 else 26.995
    sync_command = {
        'frequency': configuration['frequency'],
        'dead_frequency': dead_frequency,
        'burst_us': configuration['synchronization_burst_us'],
        'spacing_us': configuration['synchronization_spacing_us'],
        'repeats': configuration['total_synchronizations'],
    }
    base_command = {
        'frequency': configuration['frequency'],
        'dead_frequency': dead_frequency,
        'burst_us': configuration['signal_burst_us'],
        'spacing_us': configuration['signal_spacing_us'],
    }

    movement_to_command = {}
    for key in (
        'forward',
        'forward_left',
        'forward_right',
        'reverse',
        'reverse_left',
        'reverse_right',
    ):
        command_dict = base_command.copy()
        command_dict['repeats'] = configuration[key]
        movement_to_command[key] = command_dict

    # We also need to add an idle command; just broadcast at the dead frequency
    command_dict = base_command.copy()
    command_dict['frequency'] = dead_frequency
    command_dict['repeats'] = 20  # Doesn't matter
    movement_to_command['idle'] = command_dict

    direct_commands = {
        key: json.dumps([sync_command, movement_to_command[key]])
        for key in movement_to_command
    }

    return direct_commands


def get_keys():
    """Returns a tuple of (UP, DOWN, LEFT, RIGHT, changed) representing which
    keys are UP or DOWN and whether or not the key states changed.
    """
    global UP
    global DOWN
    global LEFT
    global RIGHT
    global QUIT
    change = False
    listen_keys = (pygame.K_LEFT, pygame.K_RIGHT, pygame.K_UP, pygame.K_DOWN)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            QUIT = True

        elif event.type == pygame.KEYDOWN:
            change = (event.key in listen_keys)
            if event.key == pygame.K_LEFT:
                LEFT = True
            elif event.key == pygame.K_RIGHT:
                RIGHT = True
            elif event.key == pygame.K_UP:
                UP = True
            elif event.key == pygame.K_DOWN:
                DOWN = True
            elif event.key == pygame.K_ESCAPE:
                QUIT = True

        elif event.type == pygame.KEYUP:
            change = (event.key in listen_keys)
            if event.key == pygame.K_LEFT:
                LEFT = False
            elif event.key == pygame.K_RIGHT:
                RIGHT = False
            elif event.key == pygame.K_UP:
                UP = False
            elif event.key == pygame.K_DOWN:
                DOWN = False

    return (UP, DOWN, LEFT, RIGHT, change)


def main(host, port, configuration_file_name):
    """Runs the interactive control."""
    pygame.init()
    size = (500, 100)
    screen = pygame.display.set_mode(size)
    background = pygame.Surface(screen.get_size())
    clock = pygame.time.Clock()
    black = (0, 0, 0)
    white = (255, 255, 255)
    font = pygame.font.Font(None, 36)

    pygame.display.set_caption('rc-pi interactive')

    text = font.render('Press a key to move', 1, white)
    text_position = text.get_rect(centerx=size[0] / 2)
    background.blit(text, text_position)
    screen.blit(background, (0, 0))
    pygame.display.flip()

    with open(configuration_file_name) as configuration_file:
        configuration = load_configuration(configuration_file)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while not QUIT:
        up, down, left, right, change = get_keys()

        if change:
            # Something changed, so send a new command
            command = 'idle'
            if up or down:
                if up:
                    command = 'forward'
                else:
                    command = 'reverse'

                if left:
                    command += '_left'
                elif right:
                    command += '_right'

            sock.sendto(configuration[command], (host, port))

            # Show the command and JSON
            background.fill(black)

            text = font.render(command, 1, white)
            text_position = text.get_rect(centerx=size[0] / 2)
            background.blit(text, text_position)
            screen.blit(background, (0, 0))
            pygame.display.flip()

            print(configuration[command])

        # Limit to 20 frames per second
        clock.tick(60)

    pygame.quit()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.stderr.write(
            'Usage: {program} <control JSON file>\n'.format(
                program=sys.argv[0]
            )
        )
        sys.exit(1)

    if len(sys.argv) > 2:
        HOST = sys.argv[2]
    else:
        HOST = '192.168.1.3'

    if len(sys.argv) > 3:
        PORT = int(sys.argv[3])
    else:
        PORT = 12345

    main(HOST, PORT, sys.argv[1])
