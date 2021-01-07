#!/usr/bin/python3

import math
import datetime
import pigpio
import sys
import os
import threading
from time import sleep
from subprocess import call

if len(sys.argv) < 1:
    print("Provide 1 argument:\nOffset for measure disk in degrees")
    print("Positive values indicate counterclockwise offset")
    sys.exit()

# This script must be passed one argument: measuring disk offset in degrees
offset = int(sys.argv[1])

# Serial number of CVT60 unit
unit_number = '001'

# Assign pigpio to Raspberry Pi
pi = pigpio.pi()

CW = 0                  # Clockwise stepper movement
CCW = 1                 # Counterclockwise stepper movement
enable = 0              # Enable stepper
disable = 1             # Disable stepper

servo_pin    =    27    # Servo controlling measure plate, PWM at 50Hz
stop_pin     =    2     # Stop button for halting program

pi.set_mode(servo_pin, pigpio.OUTPUT)
pi.set_PWM_frequency(servo_pin, 50)
pi.set_mode(stop_pin, pigpio.INPUT)
pi.set_pull_up_down(stop_pin, pigpio.PUD_UP)

# Loading and dispensing angles for measure servo (degrees)
load_angle = {
    1:0,
    2:31,
    3:64.8,
    4:101.3,
    5:140.2,
    }
dispense_angle = {
    1:15.5,
    2:47.9,
    3:83,
    4:120.7,
    5:160.9,
    }

def set_servo_angle(angle):
    servo_wait = 70 / 1000
    pw = angle * 2000/180 + 500
    pi.set_servo_pulsewidth(servo_pin, pw)
    sleep(servo_wait)
    
def dispense(i):
    # Load
    set_servo_angle(load_angle[i] + offset)
    sleep(1)

    # Dispense
    set_servo_angle(dispense_angle[i] + offset)
    sleep(1)

def stop_callback(gpio, level, tick):
    for i in range(20):
        sleep(0.1)
        if pi.read(27): return
            
    shutdown("STOP BUTTON PRESSED")

def shutdown(result):    
    # Release motor
    pi.set_servo_pulsewidth(servo_pin, 0)
    
    print("Shutdown complete.")
    sleep(2)
    sys.exit()


try:
    # Set callback to check for stop button press
    cb = pi.callback(stop_pin, pigpio.FALLING_EDGE, stop_callback)
    
    # Actuate dispenser on day 5 setting 5 times
    for i in range(5):
        dispense(5)

    # Actuate dispenser for each day setting
    for d in range(1,6):
        dispense(d)

    # Execute process cleanup and pass result as argument
    shutdown("SUCCESS")
    
except:
    shutdown(str(sys.exc_info()))

