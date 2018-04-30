#!/usr/bin/python3

from time import sleep
from threading import Thread
from multiprocessing import Process, Value
import sys, os

from ev3dev.ev3 import *

# Will need to check EV3 button state
btn = Button()

# Connect motors
rightMotor = LargeMotor(OUTPUT_A)
assert rightMotor.connected, "Error: Right motor not connected"
leftMotor = LargeMotor(OUTPUT_D)
assert leftMotor.connected, "Error: Left motor not connected"
# clawMotor = MediumMotor(OUTPUT_B)
# assert clawMotor.connected, "Error: Claw motor not connected"
# ultraMotor = MediumMotor(OUTPUT_C)
# assert ultraMotor.connected, "Error: Ultra motor not connected"

# Connect colour sensor
# colorSensor = ColorSensor()
# assert colorSensor.connected, "Error: Color sensor not connected"
# colorSensor.mode = "COL-COLOR"

# Connect gyro sensor
gyroSensor = GyroSensor()
assert gyroSensor.connected, "Error: Color sensor not connected"
gyroSensor.mode = 'GYRO-RATE'
gyroSensor.mode = 'GYRO-ANG' # Calibrate gyro by switching modes

# Connect ultrasonic sensor
# https://sites.google.com/site/ev3python/learn_ev3_python/using-sensors
# https://media.readthedocs.org/pdf/ev3dev-lang/latest/ev3dev-lang.pdf

#ultraSensor = UltrasonicSensor()
#assert ultraSensor.connected, "Error: Ultrasonic sensor not connected"
#ultraSensor.mode = "US-DIST-CM" # This is actually in millimetres

def brake():
    leftMotor.stop(stop_action='brake')
    rightMotor.stop(stop_action='brake')

TURN_SPEED = 750 # 0 to 1000
FINE_SPEED = 50
OFFSET = 40 # In degrees
FINE_OFFSET = 0
REVOLUTION = 360

def turn(angle):
    if (angle == 0):
        return
    direction = angle / abs(angle)
    initial = gyroSensor.value()
    final = initial + angle

    current = initial
    print("current: %d" % current)
    print("final: %d" % final)
    rightMotor.run_forever(speed_sp = -direction * TURN_SPEED)
    leftMotor.run_forever(speed_sp = direction * TURN_SPEED)

    while (current < final - OFFSET or final + OFFSET < current):
        current = gyroSensor.value()
        print("fast: %d" % current)

    rightMotor.run_forever(speed_sp = -direction * FINE_SPEED)
    leftMotor.run_forever(speed_sp = direction * FINE_SPEED)

    while (current < final - FINE_OFFSET or final + FINE_OFFSET < current):
        current = gyroSensor.value()
        print("slow: %d" % current)

    brake()

while not btn.any():
    print("Enter time you want to drive")
    y = input()
    y = int(y)
    rightMotor.run_timed(speed_sp = 1000, time_sp = y)
    leftMotor.run_timed(speed_sp = 1000, time_sp = y)
    print("Enter angle you want to turn")
    x = input()
    x = int(x)
    turn(x)

brake()
exit()
