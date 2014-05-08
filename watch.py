"""Watches the RC car for movement."""
from PIL import Image
from PIL import ImageOps
import atexit
import datetime
import itertools
import json
import math
import os
import socket
import subprocess
import sys
import time

# pylint: disable=superfluous-parens
# pylint: disable=global-statement


POPEN = None
CHANNELS_49_MHZ = (49.830, 49.845, 49.860, 49.875, 49.890)
CHANNELS_27_MHZ = (26.995, 27.045, 27.095, 27.145, 27.195, 27.255)

def terminate():
    """Terminates the program and any running background processes."""
    if POPEN is not None and POPEN.poll() is not None:
        print('Killing image capture')
        try:
            POPEN.kill()
            time.sleep(1)
        except OSError:
            pass
    sys.exit(0)

def normalize(img):
    """Linear normalization and conversion to grayscale of an image."""
    img = ImageOps.grayscale(img)
    img = ImageOps.autocontrast(img)
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

    pairs = itertools.izip(image_1.getdata(), image_2.getdata())
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
        except IOError: # proc has already terminated
            continue

    return command_lines

def format_command(
    frequency,
    useconds,
    sync_multiplier,
    sync_repeats,
    signal_repeats
):
    """Returns the JSON command string for this command tuple."""
    return json.dumps({
        'synchronization_burst_us': useconds * sync_multiplier,
        'synchronization_spacing_us': useconds,
        'total_synchronizations': sync_repeats,
        'signal_burst_us': useconds,
        'signal_spacing_us': useconds,
        'total_signals': signal_repeats,
        'frequency': frequency,
        'dead_frequency': 49.890 if frequency < 38 else 26.995,
    })


def command_iterator(frequency):
    """Iterates through the frequencies and commands."""
    for useconds in xrange(400, 1200, 100):
        for sync_multiplier in xrange(2, 7):
            for sync_repeats in xrange(2, 7):
                for signal_repeats in xrange(5, 50):
                    yield (
                        frequency,
                        useconds,
                        sync_multiplier,
                        sync_repeats,
                        signal_repeats,
                    )


def main(host, port, frequencies, crop_box=None):
    """Iterates through commands and looks for changes in the webcam."""
    if port is None:
        port = 12345
    # Remove the default image, so make sure that we're not processing images
    # from a previous run
    try:
        os.remove('photo.png')
    except OSError:
        pass

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

    diffs = []
    base = normalize(get_picture(crop_box=crop_box))
    base.save('cropped-test.png')
    time.sleep(1)
    print('Filling base photos')
    for _ in xrange(10):
        recent = normalize(get_picture(crop_box=crop_box))
        diff = percent_difference(base, recent)
        print('diff = {diff}'.format(diff=diff))
        time.sleep(1)
        diffs.append(diff)

    print('Searching for command codes')

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    for frequency in frequencies:
        for command_tuple in command_iterator(frequency):
            command = format_command(*command_tuple)
            sock.sendto(command + '\n', (host, 12345))
            time.sleep(1)

            recent = normalize(get_picture(crop_box=crop_box))
            # Let's compare the most recent photo to the oldest one, in case a
            # cloud passes over and the brightness changes
            diff = percent_difference(base, recent)
            std_dev = standard_deviation(diffs)
            mean_ = mean(diffs)
            # I should be doing a z-test or something here... eh
            if abs(diff - mean_) > (std_dev * 3.0) and diff > 5.0:
                print(
                    '{diff} at {time}'.format(
                        diff=diff,
                        time=str(datetime.datetime.now())
                    )
                )
                file_name = '-'.join(str(i) for i in command_tuple)
                os.rename('photo.png', file_name + '.png')
                time.sleep(2)

            # This is incredibly slow, and should be using a linked list or
            # something... but it's only happening once a second...
            diffs.remove(diffs[0])
            diffs.append(diff)


if __name__ == '__main__':
    atexit.register(terminate)

    if len(sys.argv) > 1:
        HOST = sys.argv[1]
    else:
        HOST = '192.168.1.3'

    if len(sys.argv) > 2:
        PORT = int(sys.argv[2])
    else:
        PORT = 12345

    try:
        main(HOST, PORT, CHANNELS_49_MHZ)
    except Exception as exc:
        print('Caught exception, exiting')
        print(str(exc))
