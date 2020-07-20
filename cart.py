#!/usr/bin/python3

import math
import time
import datetime
import pigpio
import itertools
import sys
import os
from subprocess import call
from adafruit_motorkit import MotorKit
from adafruit_motor import stepper

# Serial number of CVT60 unit
unit_number = '001'

# Angle in degrees to add to bring axes parallel with wall when homed
# (positive = toward wall, negative = away from wall)
stepper_cal_1 = 2.5
stepper_cal_2 = 2

# Number of columns (x) and rows (y) of jars on cart
jar_num_x = 11
jar_num_y = 7

# Diameter of jars in mm
jar_diam = 78

# Location of arm shoulder axis relative to jars in mm
ori_x = jar_num_x*jar_diam / 2
ori_y = -10

# Length of robot arm sections in mm
arm_1 = 330
arm_2 = 330

# Coefficient for converting degrees to steps
# (pulley tooth count/motor tooth count * steps per revolution/360)
stepper_1_deg_to_step = 116/20 * 200/360
stepper_2_deg_to_step = 80/20 * 200/360

# Time to wait between individual steps in ms / 1000
wait = 8 / 1000

pi = pigpio.pi()

# Servo (measure) connected to GPIO 21 (pin 40) and GND (pin 39) with PWM at 50Hz
servo_pin = 21
pi.set_mode(servo_pin, pigpio.OUTPUT)
pi.set_PWM_frequency(servo_pin, 50)

# DC vibration motor connected to GPIO 5 (pin 29) and GND (pin 30) with PWM at 50Hz
dc_pin = 5
pi.set_mode(dc_pin, pigpio.OUTPUT)
pi.write(dc_pin, 1)

# Limit switches connected to GPIO 18 (pin 12) and GND (pin 14)
lmt_pin = 18
pi.set_mode(lmt_pin, pigpio.INPUT)
pi.set_pull_up_down(lmt_pin, pigpio.PUD_UP)

# Stop button connected to GPIO 27 and GND
stop_pin = 27
pi.set_mode(stop_pin, pigpio.INPUT)
pi.set_pull_up_down(stop_pin, pigpio.PUD_UP)

# Initialize global stepper variables
stepper_kit = MotorKit()
stepper_1 = 0
stepper_2 = 0

# Loading and dispensing angles for measure servo
load_angle = {
    1:0,
    2:31,
    3:64.8,
    4:101.3,
    5:140.2
    }
dispense_angle = {
    1:15.5,
    2:47.9,
    3:83,
    4:120.7,
    5:160.9
    }

def get_day(i):
    switcher={
        0:5,    # Monday    (day 5)
        1:1,    # Tuesday   (demo/testing)
        2:1,    # Wednesday (demo/testing)
        3:1,    # Thursday  (day 1)
        4:2,    # Friday    (day 2)
        5:3,    # Saturday  (day 3)
        6:4     # Sunday    (day 4)
        }
    return switcher.get(i, "invalid day")

def easeinout(t):
    b = 15 / 1000   # initial wait time
    c = wait - b    # change in wait time
    d = 20          # number of steps over which to change wait time
    
    t /= d/2
    if t < 1:
        return c/2*t*t+b
    t = t-1
    return -c/2*(t*(t-2)-1)+b

def home():
    # Backup both axes before homing
    for i in range(20):
        stepper_kit.stepper1.onestep(direction=stepper.FORWARD, style=stepper.DOUBLE)
        stepper_kit.stepper2.onestep(direction=stepper.BACKWARD, style=stepper.DOUBLE)
        if i <= 20:
            time.sleep(easeinout(i))
        else:
            time.sleep(wait)        
    time.sleep(1)
    
    # Release stepper 1 while second axis is homing
    stepper_kit.stepper1.release()
    
    # Move second axis up to 370 degrees to home
    # Limit switch reads True when tripped
    for i in range(int(370*stepper_2_deg_to_step)):
        if i < (int(370*stepper_2_deg_to_step)) and pi.read(lmt_pin):
            stepper_2_homed = True
        elif i == (int(370*stepper_2_deg_to_step)):
            shutdown("STEPPER 2 HOMING FAILED")
        else:
            stepper_kit.stepper2.onestep(direction=stepper.FORWARD, style=stepper.DOUBLE)
            if i <= 20:
                time.sleep(easeinout(i))
            else:
                time.sleep(wait)    
    time.sleep(1)
    
    # Back second axis off limit switch to free circuit for first axis homing
    for i in range(20):
        stepper_kit.stepper2.onestep(direction=stepper.BACKWARD, style=stepper.DOUBLE)
        if i <= 20:
            time.sleep(easeinout(i))
        else:
            time.sleep(wait)        
    time.sleep(1)
        
    # Move first axis up to 190 degrees to home
    # Limit switch reads True when tripped
    for i in range(int(190*stepper_1_deg_to_step)):
        if i < (int(190*stepper_1_deg_to_step)) and pi.read(lmt_pin):
            stepper_1_homed = True
        elif i == (int(190*stepper_2_deg_to_step)):
            shutdown("STEPPER 1 HOMING FAILED")
        else:
            stepper_kit.stepper1.onestep(direction=stepper.BACKWARD, style=stepper.DOUBLE)
            if i <= 20:
                time.sleep(easeinout(i))
            else:
                time.sleep(wait)            
    time.sleep(1)
    
    # Return second axis to home
    for i in range(20):
        stepper_kit.stepper2.onestep(direction=stepper.FORWARD, style=stepper.DOUBLE)
        if i <= 20:
            time.sleep(easeinout(i))
        else:
            time.sleep(wait)
    time.sleep(1)

def get_coord(j, origin):
    # Add radius to get center of jar and subtract arm origin offset
    return j*jar_diam + jar_diam/2 - origin
    
def get_step_count_1(x, y):
    global stepper_1
    current_stepper_1 = stepper_1
    if x > 0:
        stepper_1 = 180 - (math.degrees(math.atan(y/x)) + math.degrees(math.acos((x*x + y*y + arm_1*arm_1 - arm_2*arm_2) / (2*math.sqrt(y*y + x*x)*arm_1))))
    elif x < 0:
        stepper_1 = math.degrees(math.atan(y/abs(x))) + math.degrees(math.acos((x*x + y*y + arm_1*arm_1 - arm_2*arm_2) / (2*math.sqrt(y*y + x*x)*arm_1)))
    elif x == 0:
        stepper_1 = math.degrees(math.atan(y/1)) + math.degrees(math.acos((1 + y*y + arm_1*arm_1 - arm_2*arm_2) / (2*math.sqrt(y*y + 1)*arm_1)))    
    
    step_count = stepper_1_deg_to_step * (current_stepper_1 - stepper_1)
    
    print("x=" + str(x) + ", y=" + str(y))
    print("angle_1=" + str(stepper_1))
    
    return int(step_count)

def get_step_count_2(x, y):
    global stepper_2
    current_stepper_2 = stepper_2
    if x > 0:
        stepper_2 = math.degrees(math.acos((arm_2*arm_2 + arm_1*arm_1 - x*x - y*y) / (2*arm_1*arm_2)))
    elif x <= 0:
        stepper_2 = 360 - math.degrees(math.acos((arm_2*arm_2 + arm_1*arm_1 - x*x - y*y) / (2*arm_1*arm_2)))
    
    step_count = stepper_2_deg_to_step * (current_stepper_2 - stepper_2)
    
    print("angle_2=" + str(stepper_2))
    
    return int(step_count)
        
def start_steps(step_count_1, step_count_2):
    i = 0
    m = max(abs(step_count_1), abs(step_count_2))
    
    for step_1, step_2 in itertools.zip_longest(range(abs(step_count_1)), range(abs(step_count_2))):
        if step_count_1 > 0 and step_1 != None:
            stepper_kit.stepper1.onestep(direction=stepper.BACKWARD, style=stepper.DOUBLE)
        elif step_count_1 < 0 and step_1 != None:
            stepper_kit.stepper1.onestep(direction=stepper.FORWARD, style=stepper.DOUBLE)
        if step_count_2 > 0 and step_2 != None:
            stepper_kit.stepper2.onestep(direction=stepper.FORWARD, style=stepper.DOUBLE)
        elif step_count_2 < 0 and step_2 != None:
            stepper_kit.stepper2.onestep(direction=stepper.BACKWARD, style=stepper.DOUBLE)
        
        # Ease into and out of movement
        i += 1
        if i <= 20:
            time.sleep(easeinout(i))
        elif i >= (m - 20):
            time.sleep(easeinout(m - i))
        else:
            time.sleep(wait)

def set_servo_angle(servo, angle):
    servo_wait = 70 / 1000              # Time between shaking movements in milliseconds
    pw = angle * 2000/180 + 500
    pi.set_servo_pulsewidth(servo, pw)
    time.sleep(servo_wait)
    
def vibrate(seconds):
    pi.write(dc_pin, 0)
    time.sleep(seconds)
    pi.write(dc_pin, 1)
    time.sleep(0.5)
    
def dispense(i):
    # Load
    set_servo_angle(load_angle[i] + offset)
    time.sleep(0.5)
    vibrate(1)
    # Dispense
    set_servo_angle(dispense_angle[i] + offset)
    time.sleep(0.25)
    vibrate(1)

def stop_callback(gpio, level, tick):
    for i in range(10):
        time.sleep(0.1)
        if pi.read(27):
            return
    shutdown("STOP BUTTON PRESSED")

def shutdown(result):
    print("Shutting down")
    # Release steppers
    stepper_kit.stepper1.release()
    stepper_kit.stepper2.release()
    # Release servos
    #TODO update motors and release DC motor
    pi.set_servo_pulsewidth(servo_pin, 0)
    pi.write(dc_pin, 1)
    # End servo PWM
    pi.stop()
    
    # Print report
    print(str(datetime.datetime.now()) + ": " + result)
    file = open("/home/pi/Desktop/log.txt", "a+")
    file.write("\n" + str(datetime.datetime.now()) + ": " + result)
    file.close()
    
    # Log report on Google Sheets
    call(['/usr/bin/python3', 'logger.py', unit_number, result])
    
    time.sleep(2)
    # Exit script
    sys.exit()

def start_position(x,y):
    # Get next x/y coordinates of jar
    x_coord = get_coord(x, ori_x)
    y_coord = get_coord(y, ori_y)
    # Get number of steps (positive or negative) to next coordinates
    step_count_1 = get_step_count_1(x_coord, y_coord)
    step_count_2 = get_step_count_2(x_coord, y_coord)
    # Start steppers
    start_steps(step_count_1, step_count_2)

try:
    # Callback to check for stop button press
    cb = pi.callback(27, pigpio.FALLING_EDGE, stop_callback)
    
    # Get current feeding day
    day = get_day(datetime.date.today().weekday())

    # Home motors before beginning
    home()

    # Go to predefined start position (x,y) before continuing cycle
    # This is implemented to avoid dispenser hitting wall on the way to jar(0,0)
    start_position(4,4)
    time.sleep(0.5)
    # Add calibration adjustment to both axes
    start_steps(int(stepper_cal_1*stepper_1_deg_to_step), int(stepper_cal_2*stepper_2_deg_to_step))
    time.sleep(0.5)

    # Run main dispensing procedure
    for x in range(jar_num_x):
        if x % 2 == 0:
            for y in range(jar_num_y):
                # Get next x/y coordinates of jar
                x_coord = get_coord(x, ori_x)
                y_coord = get_coord(y, ori_y)
                # Get number of steps (positive or negative) to next coordinates
                step_count_1 = get_step_count_1(x_coord, y_coord)
                step_count_2 = get_step_count_2(x_coord, y_coord)
                # Start steppers
                start_steps(step_count_1, step_count_2)
                dispense(day) # Accepts day integer 1-5
                print("---------------------------------------------")
        else:
            # Reverse order for odd-numbered columns
            for y in range(jar_num_y-1, -1, -1):
                # Get next x/y coordinates of jar
                x_coord = get_coord(x, ori_x)
                y_coord = get_coord(y, ori_y)
                # Get number of steps (positive or negative) to next coordinates
                step_count_1 = get_step_count_1(x_coord, y_coord)
                step_count_2 = get_step_count_2(x_coord, y_coord)
                # Start steppers
                start_steps(step_count_1, step_count_2)
                dispense(day) # Accepts day integer 1-5
                print("---------------------------------------------")

    # Return motors to home position and shutdown
    home()
    shutdown("SUCCESS")
    
except:
    shutdown(str(sys.exc_info()))

