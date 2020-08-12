#!/usr/bin/python3

import pigpio
from time import sleep
import os
import subprocess

enable = 0          # Enable stepper
disable = 1         # Disable stepper

pi = pigpio.pi()

run_pin     =   02  # Run button
sd_pin      =   03  # Shutdown button
led_pin     =   16  # Status LED
ena_pin_1   =   26  # First axis stepper enable (pin is default high)
ena_pin_2   =   19  # Second axis stepper enable (pin is default high)
servo_pin   =   27  # Servo controlling measure plate, PWM at 50Hz
dc_pin      =   04  # DC vibration motor

# Set up pins and ensure motors are disabled at startup
pi.set_mode(run_pin, pigpio.INPUT)
pi.set_pull_up_down(run_pin, pigpio.PUD_UP)
pi.set_mode(sd_pin, pigpio.INPUT)
pi.set_pull_up_down(sd_pin, pigpio.PUD_UP)
pi.set_mode(led_pin, pigpio.OUTPUT)
pi.set_mode(ena_pin_1, pigpio.OUTPUT)
pi.write(ena_pin_1, disable)
pi.set_mode(ena_pin_2, pigpio.OUTPUT)
pi.write(ena_pin_2, disable)
pi.set_mode(servo_pin, pigpio.OUTPUT)
pi.set_servo_pulsewidth(servo_pin, 0)
pi.set_mode(dc_pin, pigpio.OUTPUT)
pi.write(dc_pin, disable)

def shutdown_callback(gpio, level, tick):
    for i in range(5):
        sleep(0.1)
        if pi.read(sd_pin):
            return
    pi.write(led_pin, 0)
    pi.stop()
    os.system("sudo shutdown now -h")

def run_callback(gpio, level, tick):
    for i in range(5):
        sleep(0.1)
        if pi.read(run_pin):
            return
    subprocess.call(['/usr/bin/python3', '/home/pi/cvt60/cart.py'])

# Set button callbacks
cb1 = pi.callback(sd_pin, pigpio.FALLING_EDGE, shutdown_callback)
cb2 = pi.callback(run_pin, pigpio.FALLING_EDGE, run_callback)

while True:
    pi.write(led_pin, 1)
    sleep(0.1)
    pi.write(led_pin, 0)
    sleep(3.9)

