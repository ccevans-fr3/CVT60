#!/usr/bin/python3

import pigpio
from time import sleep
import os
import subprocess
from adafruit_motorkit import MotorKit
from adafruit_motor import stepper

pi = pigpio.pi()
pi.set_mode(27, pigpio.INPUT)   # Run button
pi.set_pull_up_down(27, pigpio.PUD_UP)
pi.set_mode(4, pigpio.INPUT)    # Shutdown button
pi.set_pull_up_down(4, pigpio.PUD_UP)
pi.set_mode(16, pigpio.OUTPUT)   # Ready LED

def shutdown_callback(gpio, level, tick):
    for i in range(10):
        sleep(0.1)
        if pi.read(4):
            return
    pi.write(16, 0)
    pi.stop()
    os.system("sudo shutdown now -h")

def run_callback(gpio, level, tick):
    for i in range(10):
        sleep(0.1)
        if pi.read(27):
            return
    subprocess.call(['/usr/bin/python3', '/home/pi/cvt60/cart.py'])

# Set button callbacks
cb1 = pi.callback(4, pigpio.FALLING_EDGE, shutdown_callback)
cb2 = pi.callback(27, pigpio.FALLING_EDGE, run_callback)

# Ensure steppers are released on startup
stepper_kit = MotorKit()
stepper_kit.stepper1.release()
stepper_kit.stepper2.release()

while True:
    pi.write(16, 1)
    sleep(0.1)
    pi.write(16, 0)
    sleep(3.9)

