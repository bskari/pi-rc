#!/usr/bin/env python
"""Interactive control for the remote radio control Raspberry Pi.  Script
modified by Jack Davies (lavanoid) to work with the PiFace CAD.
"""

# To use this script, you must install the PiFace CAD library using "sudo
# apt-get install python-pifacecad"
import argparse
import json
import time
import pifacecad
cad = pifacecad.PiFaceCAD()
import socket
from common import dead_frequency
from common import server_up

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
        'reverse',
        'reverse_left',
        'reverse_right',
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

def interactive_control(host, port, configuration):
    """Runs the interactive control."""
    cad.lcd.backlight_off() # Save power. Useful for battery projects.
    controlname = 'Pi-RC - 2Q, 3RE'
    cad.lcd.write(controlname + "\n0L,1LED,5A,   4R")
    cad.lcd.cursor_off()
    globals()['sock'] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    globals()['host'] = host
    globals()['port'] = port
    globals()['configuration'] = configuration
    command = 'idle'
    currentcommand = command
    operation = 'forward'
    backlight = False
    while True:
        command = 'idle'
        # Use the joystick key to change the acceleration direction.
        if cad.switches[1].value == 1:
            if backlight == False:
                backlight = True
                cad.lcd.backlight_on()
                time.sleep(1)
            else:
                backlight = False
                cad.lcd.backlight_off()
                time.sleep(1)
        if cad.switches[6].value == 1:
            operation = 'reverse'
        else:
			if cad.switches[7].value == 1:
				operation = 'forward'
			elif cad.switches[3].value == 1:
				operation = 'reverse'
        if cad.switches[2].value == 1:
			# command is currently idle.
            sock.sendto(configuration[command], (host, port))
            exit()
        else:
			if cad.switches[5].value == 1:
				command = operation
			elif cad.switches[3].value == 1:
				command = operation
			if command == 'forward' or command == 'reverse':
				if cad.switches[0].value == 1:
					command += '_left'
				elif cad.switches[4].value == 1:
					command += '_right'
			if not command == currentcommand:
				# Avoid causing pi_pcm from crashing.
				currentcommand = command
				sock.sendto(configuration[command], (host, port))


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
