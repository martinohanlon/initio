import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

GPIO.setup(24,GPIO.IN)

while True:
    print GPIO.input(24)
    time.sleep(0.1)
