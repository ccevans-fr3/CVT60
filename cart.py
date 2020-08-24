#!/usr/bin/python3

import math
import time
import datetime
import pigpio
import sys
import os
import threading
from subprocess import call

# Serial number of CVT60 unit
unit_number = '001'

# Step adjustment to bring axes parallel with wall when homed
# (positive = CW, negative = CCW)
stepper_cal_1 = 2
stepper_cal_2 = -2

# Number of microsteps per full step (e.g. half step = 2)
step_mode = 4

# Number of columns (x) and rows (y) of jars on cart
jar_num_x = 11
jar_num_y = 7

# Diameter of jars in mm
jar_diam = 78

# Location of arm shoulder axis relative to jars in mm
ori_x = jar_num_x*jar_diam / 2
ori_y = -6

# Length of arm sections in mm
arm_1 = 330
arm_2 = 330

# Coefficient for converting degrees to steps
# (pulley tooth count/motor tooth count * steps per revolution/360)
stepper_1_deg_to_step = 116/20 * 200/360 * step_mode
stepper_2_deg_to_step = 80/20 * 200/360 * step_mode

# Time to wait between individual steps in ms / 1000
wait = (8 / step_mode) / 1000

# Number of steps over which to implement easing function
ease_count = 20 * step_mode

pi = pigpio.pi()

# Initialize stepper positions
stepper_1 = 0
stepper_2 = 0

CW = 0                  # Clockwise stepper movement
CCW = 1                 # Counterclockwise stepper movement
enable = 0              # Enable stepper
disable = 1             # Disable stepper

# Pin assignments. All numbers are BCM, not physical pin number.
# TODO consider using single enable pin
step_pin_1   =    21    # First axis stepper movement
dir_pin_1    =    13    # First axis stepper direction
ena_pin_1    =    26    # First axis stepper enable (pin is default high)
step_pin_2   =    12    # Second axis stepper movement
dir_pin_2    =    20    # Second axis stepper direction
ena_pin_2    =    19    # Second axis stepper enable (pin is default high)

servo_pin    =    27    # Servo controlling measure plate, PWM at 50Hz
dc_pin       =    4     # DC vibration motor
lmt_pin_1    =    23    # Limit switch for homing first axis
lmt_pin_2    =    22    # Limit switch for homing second axis
stop_pin     =    2     # Stop button for halting program

pi.set_mode(step_pin_1, pigpio.OUTPUT)
pi.write(step_pin_1, 0)
pi.set_mode(dir_pin_1, pigpio.OUTPUT)
pi.write(dir_pin_1, 0)
pi.set_mode(ena_pin_1, pigpio.OUTPUT)
pi.write(ena_pin_1, enable)
pi.set_mode(step_pin_2, pigpio.OUTPUT)
pi.write(step_pin_2, 0)
pi.set_mode(dir_pin_2, pigpio.OUTPUT)
pi.write(dir_pin_2, 0)
pi.set_mode(ena_pin_2, pigpio.OUTPUT)
pi.write(ena_pin_2, enable)

pi.set_mode(servo_pin, pigpio.OUTPUT)
pi.set_PWM_frequency(servo_pin, 50)
pi.set_mode(dc_pin, pigpio.OUTPUT)
pi.write(dc_pin, disable)
pi.set_mode(lmt_pin_1, pigpio.INPUT)
pi.set_pull_up_down(lmt_pin_1, pigpio.PUD_UP)
pi.set_mode(lmt_pin_2, pigpio.INPUT)
pi.set_pull_up_down(lmt_pin_2, pigpio.PUD_UP)
pi.set_mode(stop_pin, pigpio.INPUT)
pi.set_pull_up_down(stop_pin, pigpio.PUD_UP)

# Loading and dispensing angles for measure servo
offset = 6          # Offset for measuring disc
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

def get_day(i):
    switcher={
        0:5,    # Monday    (day 5)
        1:1,    # Tuesday   (demo/testing)
        2:1,    # Wednesday (demo/testing)
        3:1,    # Thursday  (day 1)
        4:2,    # Friday    (day 2)
        5:3,    # Saturday  (day 3)
        6:4,    # Sunday    (day 4)
        }
    return switcher.get(i, "invalid day")

def easeinout(t):
    """
    Quadratic ease in/out function.
    This eases the steppers up to full speed and back to rest
    to reduce strain and prevent missed steps.
    """

    b = wait*4      # initial wait time
    c = wait - b    # change in wait time
    d = ease_count  # number of steps over which to change wait time
    
    t /= d/2
    if t < 1:
        return c/2*t*t+b
    t = t-1
    return -c/2*(t*(t-2)-1)+b

def home():
    """
    Homing function.
    The CVT uses limit switches on a single circuit to find home position.
    Once one arm is homed, it must back off the limit switch to open the
    circuit before homing the next arm.
    """

    backup_degrees = -10    # Degrees to back up each arm
    axis_1_degrees = 190    # Degrees to move first axis before failing
    axis_2_degrees = 370    # Degrees to move second axis before failing
    #stepper_1_homed = False
    #stepper_2_homed = False

    # Backup both axes before homing    
    start_steps(int(backup_degrees*stepper_1_deg_to_step), 
                int(backup_degrees*stepper_2_deg_to_step))

    time.sleep(1)
    
    # Release stepper 1 while second axis is homing
    pi.write(ena_pin_1, disable)
    
    # Home second axis
    for i in range(int(axis_2_degrees*stepper_2_deg_to_step)):
        if i < (int(axis_2_degrees*stepper_2_deg_to_step)) and pi.read(lmt_pin_2):
            stepper_2_homed = True
        elif i == (int(axis_2_degrees*stepper_2_deg_to_step))-1:
            shutdown("STEPPER 2 HOMING FAILED")
        else:
            step(step_pin_2, CCW)
            time.sleep(wait)    # This adds to step wait time, slowing motor for homing
   
    time.sleep(1)

    # Re-enable stepper 1
    pi.write(ena_pin_1, enable)

    # Home first axis
    for i in range(int(axis_1_degrees*stepper_1_deg_to_step)):
        if i < (int(axis_1_degrees*stepper_1_deg_to_step)) and pi.read(lmt_pin_1):
            stepper_1_homed = True
        elif i == (int(axis_1_degrees*stepper_2_deg_to_step))-1:
            shutdown("STEPPER 1 HOMING FAILED")
        else:
            step(step_pin_1, CW)
            time.sleep(wait)    # This adds to step wait time, slowing motor for homing

    time.sleep(1)

def goto_coords(x, y):
    """
    Initiate stepper movement.
    The coordinates of the next jar in mm is calculated, then this
    is passed to the function to calculate the number of stepper motor
    steps to reach this position.
    """

    # Add radius to get center of jar and subtract arm origin offset
    x_coord = x*jar_diam + jar_diam/2 - ori_x
    y_coord = y*jar_diam + jar_diam/2 - ori_y
    get_step_counts(x_coord, y_coord)

def get_step_counts(x, y):
    """
    A reference is held to the current angle of steppers in degrees.
    The next stepper positions are calculated from the jar coordinates
    in mm and the difference between angles calculated.
    """

    global stepper_1, stepper_2
    current_stepper_1 = stepper_1
    current_stepper_2 = stepper_2
    
    if x > 0:
        stepper_1 = 180 - (math.degrees(math.atan(y/x)) \
                    + math.degrees(math.acos((x*x + y*y + arm_1*arm_1 - arm_2*arm_2) \
                    / (2*math.sqrt(y*y + x*x)*arm_1))))
        stepper_2 = math.degrees(math.acos((arm_2*arm_2 + arm_1*arm_1 - x*x - y*y) \
                    / (2*arm_1*arm_2)))
    elif x < 0:
        stepper_1 = math.degrees(math.atan(y/abs(x))) \
                    + math.degrees(math.acos((x*x + y*y + arm_1*arm_1 - arm_2*arm_2) \
                    / (2*math.sqrt(y*y + x*x)*arm_1)))
        stepper_2 = 360 - math.degrees(math.acos((arm_2*arm_2 + arm_1*arm_1 - x*x - y*y) \
                    / (2*arm_1*arm_2)))
    elif x == 0:
        stepper_1 = math.degrees(math.atan(y/1)) \
                    + math.degrees(math.acos((1 + y*y + arm_1*arm_1 - arm_2*arm_2) \
                    / (2*math.sqrt(y*y + 1)*arm_1)))
        stepper_2 = 360 - math.degrees(math.acos((arm_2*arm_2 + arm_1*arm_1 - x*x - y*y) \
                    / (2*arm_1*arm_2)))
    
    step_count_1 = int(stepper_1_deg_to_step * (current_stepper_1 - stepper_1))
    step_count_2 = int(stepper_2_deg_to_step * (current_stepper_2 - stepper_2))
    
    print("x=" + str(x) + ", y=" + str(y))
    print("angle_1=" + str(stepper_1) + ", angle_2=" + str(stepper_2))
    print("---------------------------------------------")
    
    start_steps(step_count_1, step_count_2)

def step(stepper, direction):
    if stepper is step_pin_1:
        pi.write(dir_pin_1, direction)
    elif stepper is step_pin_2:
        pi.write(dir_pin_2, direction)
    
    pi.write(stepper, 1)
    time.sleep(wait/2)
    pi.write(stepper, 0)
    time.sleep(wait/2)

def step_thread(stepper, step_count):
    i = 0   # Current step number

    for s in range(abs(step_count)):
        if stepper == 1:
            if step_count > 0:
                step(step_pin_1, CW)
            elif step_count < 0:
                step(step_pin_1, CCW)
        elif stepper == 2:
            if step_count > 0:
                step(step_pin_2, CW)
            elif step_count < 0:
                step(step_pin_2, CCW)
        
        # Increment step counter
        i += 1 
        
        # Ease into and out of movement
        if i <= ease_count and i < abs(step_count)/2:
            time.sleep(easeinout(i)-wait)
        elif i >= abs(step_count)-ease_count:
            time.sleep(easeinout(abs(step_count)-i)-wait)
        
def start_steps(step_count_1, step_count_2):
    # Create stepper threads
    t1 = threading.Thread(target=step_thread, args=(1, step_count_1))
    t2 = threading.Thread(target=step_thread, args=(2, step_count_2))

    # Start stepper threads
    t1.start()
    t2.start()

    # Wait until both threads have finished
    t1.join()
    t2.join()

def set_servo_angle(angle):
    servo_wait = 70 / 1000
    pw = angle * 2000/180 + 500
    pi.set_servo_pulsewidth(servo_pin, pw)
    time.sleep(servo_wait)
    
def vibrate(seconds):
    pi.write(dc_pin, enable)
    time.sleep(seconds)
    pi.write(dc_pin, disable)
    time.sleep(0.5)
    
def dispense(i):
    # Load
    set_servo_angle(load_angle[i] + offset)
    time.sleep(0.5)
    vibrate(1)

    # Dispense
    set_servo_angle(dispense_angle[i] + offset)
    time.sleep(0.5)
    vibrate(1)

def stop_callback(gpio, level, tick):
    for i in range(5):
        time.sleep(0.1)
        if pi.read(27):
            return
            
    shutdown("STOP BUTTON PRESSED")

def shutdown(result):
    print("Shutting down...")
    
    # Release motors
    pi.write(ena_pin_1, disable)
    pi.write(ena_pin_2, disable)
    pi.set_servo_pulsewidth(servo_pin, 0)
    pi.write(dc_pin, disable)
    pi.stop()
    
    # Print report
    print(str(datetime.datetime.now()) + ": " + result)
    file = open("/home/pi/Desktop/log.txt", "a+")
    file.write("\n" + str(datetime.datetime.now()) + ": " + result)
    file.close()
    
    # Log report on Google Sheets
    call(['/usr/bin/python3', 'logger.py', unit_number, result])
    
    print("Shutdown complete.")
    time.sleep(2)
    sys.exit()


try:
    # Setup callback to check for stop button press
    cb = pi.callback(stop_pin, pigpio.FALLING_EDGE, stop_callback)
    
    # Get current feeding day
    day = get_day(datetime.date.today().weekday())

    # Home motors before beginning
    home()
    
    # Add calibration adjustment to both axes
    start_steps(stepper_cal_1, stepper_cal_2)
    time.sleep(1)
    
    # Go to predefined start position (x,y) before continuing cycle
    # This is implemented to avoid dispenser hitting wall on the way to jar(0,0)
    goto_coords(4, 4)

    # Run main dispensing procedure
    for x in range(jar_num_x):
        if x % 2 == 0:
            for y in range(jar_num_y):              # Forward for even-numbered columns
                goto_coords(x, y)                   # Goto x/y coordinates of next jar
                dispense(day)                       # Accepts day integer 1-5
        else:
            for y in range(jar_num_y-1, -1, -1):    # Reverse for odd-numbered columns
                goto_coords(x, y)                   # Go to x/y coordinates of next jar
                dispense(day)                       # Accepts day integer 1-5

    # Return steppers to home position
    home()
    
    # Execute process cleanup and pass result as argument
    shutdown("SUCCESS")
    
except:
    shutdown(str(sys.exc_info()))

