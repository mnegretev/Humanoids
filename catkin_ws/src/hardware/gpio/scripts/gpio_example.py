#!/usr/bin/env python3

try:
    import RPi.GPIO as GPIO
    import time
except RuntimeError:
    print("Error importing")


GPIO.setmode(GPIO.BOARD)

GPIO.setup(36, GPIO.IN)


for i in range(5):
    print(GPIO.input(36))
    time.sleep(1)

GPIO.cleanup(36)