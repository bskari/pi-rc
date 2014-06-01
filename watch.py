"""Watches the RC car for movement."""
from PIL import Image
from PIL import ImageOps
from collections import deque
import argparse
import atexit
import datetime
import math
import os
import socket
import subprocess
import sys
import time

from common import format_command
from common import server_up

# pylint: disable=superfluous-parens
# pylint: disable=global-statement


POPEN = None
CHANNELS_49_MHZ = (49.830, 49.845, 49.860, 49.875, 49.890)
CHANNELS_27_MHZ = (26.995, 27.045, 27.095, 27.145, 27.195, 27.255)


def terminate():
    """Terminates the program and any running background processes."""
    if POPEN is not None and POPEN.poll() is None:
        print('Killing image capture')
        try:
            POPEN.kill()
            time.sleep(1)
        except OSError:
            pass

    try:
        os.remove('photo.png')
    except OSError:
        pass



def normalize(img, bit_depth=None):
    """Linear normalization and conversion to grayscale of an image."""
    img = ImageOps.grayscale(img)
    img = ImageOps.autocontrast(img)
    if bit_depth is not None:
        img = ImageOps.posterize(img, bit_depth)
    return img


def mean(values):
    """Calculate mean of the values."""
    return sum(values) / len(values)


def standard_deviation(values, mean_=None):
    """Calculate standard deviation."""
    if mean_ is None:
        mean_ = mean(values)
    size = len(values)
    sum_ = 0.0
    for value in values:
        sum_ += math.sqrt((value - mean_) ** 2)
    return math.sqrt((1.0 / (size - 1)) * (sum_ / size))


def get_picture(file_name=None, crop_box=None):
    """Saves a picture from the webcam."""
    if file_name is None:
        file_name = 'photo.png'

    image = Image.open(file_name)
    if crop_box is not None:
        image = image.crop(crop_box)
    return image


def percent_difference(image_1, image_2):
    """Returns the percent difference between two images."""
    assert image_1.mode == image_2.mode, 'Different kinds of images.'
    assert image_1.size == image_2.size, 'Different sizes.'

    pairs = zip(image_1.getdata(), image_2.getdata())
    if len(image_1.getbands()) == 1:
        # for gray-scale jpegs
        diff = sum(abs(p1 - p2) for p1, p2 in pairs)
    else:
        diff = sum(abs(c1 - c2) for p1, p2 in pairs for c1, c2 in zip(p1, p2))

    ncomponents = image_1.size[0] * image_1.size[1] * 3
    return (diff / 255.0 * 100.0) / ncomponents


def get_process_command_lines():
    """Returns all of the command lines of running proceses on the system."""
    pids = [pid for pid in os.listdir('/proc') if pid.isdigit()]

    command_lines = []
    for pid in pids:
        try:
            command_lines.append(
                open(os.path.join('/proc', pid, 'cmdline'), 'rb').read()
            )
        except IOError:  # proc has already terminated
            continue

    return command_lines


def command_iterator(frequency):
    """Iterates through the frequencies and commands."""
    for useconds in range(100, 1201, 100):
        for sync_multiplier in range(2, 7):
            for sync_repeats in range(2, 7):
                for signal_repeats in range(5, 50):
                    yield (
                        frequency,
                        useconds,
                        sync_multiplier,
                        sync_repeats,
                        signal_repeats,
                    )


def start_image_capture_process():
    """Starts the image capture background process."""
    image_capture_command_parts = [
        '/usr/bin/gst-launch-0.10', '-e',
        'v4l2src',
        '!', 'video/x-raw-rgb,width=640,height=480',
        '!', 'videorate',
        '!', 'video/x-raw-rgb,framerate=9/10',
        '!', 'ffmpegcolorspace',
        '!', 'pngenc', 'snapshot=false',
        '!', 'multifilesink', 'location=photo.png',
    ]
    image_capture_command = ' '.join(image_capture_command_parts)
    if image_capture_command not in get_process_command_lines():
        print('Starting {command}'.format(command=image_capture_command))
        global POPEN
        POPEN = subprocess.Popen(image_capture_command_parts)
        time.sleep(5)


def search_for_command_codes(host, port, frequencies, crop_box=None, bit_depth=None):
    """Iterates through commands and looks for changes in the webcam."""
    diffs = deque()
    pictures = deque()

    base = normalize(get_picture(crop_box=crop_box), bit_depth=bit_depth)
    base.save('normalized-test.png')
    time.sleep(1)
    print('Filling base photos for difference analysis')
    for _ in range(20):
        recent = normalize(get_picture(crop_box=crop_box), bit_depth=bit_depth)
        diff = percent_difference(base, recent)
        time.sleep(1)
        diffs.append(diff)
        pictures.append(recent)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(
        'Searching for command codes on {frequencies}'.format(
            frequencies=', '.join((str(f) for f in frequencies))
        )
    )
    for frequency in frequencies:
        for command_tuple in command_iterator(frequency):
            # pylint: disable=star-args
            command = format_command(*command_tuple)
            if sys.version_info.major == 3:
                command = bytes(command, 'utf-8')
            sock.sendto(command, (host, port))
            time.sleep(1)

            recent = normalize(
                get_picture(crop_box=crop_box),
                bit_depth=bit_depth
            )
            # Let's compare the most recent photo to the oldest one, in case a
            # cloud passes over and the brightness changes
            diff = percent_difference(pictures[0], recent)
            std_dev = standard_deviation(diffs)
            mean_ = mean(diffs)
            # I should be doing a z-test or something here... eh
            if abs(diff - mean_) > (std_dev * 3.0) and diff > 2.0:
                print('Found substantially different photo, saving...')
                print(
                    'diff={diff}, mean={mean}, std dev={std_dev}'
                    ' at {time}'.format(
                        diff=diff,
                        mean=mean_,
                        std_dev=std_dev,
                        time=str(datetime.datetime.now())
                    )
                )
                file_name = '-'.join(str(i) for i in command_tuple)
                os.rename('photo.png', file_name + '.png')
                time.sleep(2)

            diffs.popleft()
            diffs.append(diff)
            pictures.popleft()
            pictures.append(recent)


def make_parser():
    """Builds and returns an argument parser."""
    parser = argparse.ArgumentParser(
        description='Iterates through and broadcasts command codes and'
            ' monitors the webcam to watch for movement from the RC car.'
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
        help='The frequency to broadcast commands on.',
        default=49,
        type=float
    )

    def bit_depth_checker(bit_depth):
        """Checks that the bit depth argument is valid."""
        try:
            bit_depth = int(bit_depth)
        except:
            raise argparse.ArgumentTypeError('Bit depth must be an int')

        if not 1 <= bit_depth <= 8:
            raise argparse.ArgumentTypeError(
                'Bit depth must be between 1 and 8 inclusive'
            )

        return bit_depth

    parser.add_argument(
        '-b',
        '--bit-depth',
        dest='bit_depth',
        help='The bit depth to reduce images to.',
        type=bit_depth_checker,
        default=1
    )

    return parser


def main():
    """Parses command line arguments and runs the interactive controller."""
    parser = make_parser()
    args = parser.parse_args()

    atexit.register(terminate)

    # Remove the default image to make sure that we're not processing images
    # from a previous run
    try:
        os.remove('photo.png')
    except OSError:
        pass

    if not server_up(args.server, args.port, args.frequency):
        print('Server does not appear to be listening for messages, aborting')
        return

    start_image_capture_process()

    # RC cars in the 27 and 49 MHz spectrum typically operate on one of a
    # several channels in that frequency, but most toy RC cars that I've
    # seen only list the major frequency on the car itself. If someone
    # enters a major frequency, search each channel.
    if args.frequency == 49:
        frequencies = CHANNELS_49_MHZ
    elif args.frequency == 27:
        frequencies = CHANNELS_27_MHZ
    else:
        frequencies = [args.frequency]

    print('Sending commands to ' + args.server + ':' + str(args.port))
    try:
        search_for_command_codes(
                args.server,
                args.port,
                frequencies,
                bit_depth=args.bit_depth
        )
    # pylint: disable=broad-except
    except Exception as exc:
        print('Caught exception, exiting')
        print(str(exc))


if __name__ == '__main__':
    main()
