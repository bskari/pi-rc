from RPi import GPIO

def main(pin):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pin, GPIO.OUT)
    GPIO.setup(pin, True)
    GPIO.setup(pin, False)
    GPIO.cleanup()


if __name__ == '__main__':
    main(7)
