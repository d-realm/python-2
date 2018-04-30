#!/usr/bin/python3

from time import sleep

from ev3dev.ev3 import *

# Will need to check EV3 button state
btn = Button()

# Connect motors
rightMotor = LargeMotor(OUTPUT_A)
assert rightMotor.connected, "Error: Right motor not connected"
leftMotor  = LargeMotor(OUTPUT_D)
assert leftMotor.connected, "Error: Left motor not connected"


DRIVE_SPEED = 500
REVOLUTION = 360
DIAMETER = 5.6
PI = 3.14

rightMotor.reset() # Set the current angle to 0
leftMotor.reset()
rightMotor.stop_action = "brake"
leftMotor.stop_action = "brake"

# Distance in centimetres
def drive(distance):
    angle = distance * REVOLUTION / (DIAMETER * PI)
    rightMotor.run_forever(speed_sp = DRIVE_SPEED)
    leftMotor.run_forever(speed_sp = DRIVE_SPEED)
    while (leftMotor.position < angle):
        sleep(0.02)
    rightMotor.stop()
    leftMotor.stop()

while not (btn.any()):
    x = int(input())
    drive(x)
