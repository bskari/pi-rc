"""Stops the Raspberry Pi from broadcasting a radio signal."""
from RPi import GPIO

def main(pin):
    """Stops the Raspberry Pi from broadcasting a radio signal from the given
    pin.
    """
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pin, GPIO.OUT)
    GPIO.setup(pin, True)
    GPIO.setup(pin, False)
    GPIO.cleanup()


if __name__ == '__main__':
    main(7)
