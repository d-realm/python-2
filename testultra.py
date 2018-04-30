#!/usr/bin/python3

from time import sleep

from ev3dev.ev3 import *

ultraMotor = MediumMotor(OUTPUT_C)
assert ultraMotor.connected, "Error: Ultra motor not connected"
ultraMotor.reset() # Set the current angle to 0
ultraMotor.stop_action = "brake"

# Will need to check EV3 button state
btn = Button()

# Connect ultrasonic sensor
# https://sites.google.com/site/ev3python/learn_ev3_python/using-sensors
# https://media.readthedocs.org/pdf/ev3dev-lang/latest/ev3dev-lang.pdf
ultraSensor = UltrasonicSensor()
assert ultraSensor.connected, "Error: Ultrasonic sensor not connected"
ultraSensor.mode = "US-DIST-CM" # This is actually in millimetres

SPEED = 360

def scan_turret():
    ultraMotor.position_sp = 0
    ultraMotor.run_to_abs_pos(speed_sp = SPEED)
    while any(ultraMotor.state): # Wait until finished rotating
        sleep(0.02)
    front = ultraSensor.value() // 42

    ultraMotor.position_sp = -90
    ultraMotor.run_to_abs_pos(speed_sp = SPEED)
    while any(ultraMotor.state): # Wait until finished rotating
        sleep(0.02)
    left = ultraSensor.value() // 42

    ultraMotor.position_sp = 90
    ultraMotor.run_to_abs_pos(speed_sp = SPEED)
    while any(ultraMotor.state): # Wait until finished rotating
        sleep(0.02)
    right = ultraSensor.value() // 42

    print("front walls: %d" % front)
    print("left  walls: %d" % left)
    print("right walls: %d" % right)

while not (btn.any()):
    x = int(input())
    scan_turret()
