#!/usr/bin/python3

from time import sleep
import sys, os

from ev3dev.ev3 import *

# Will need to check EV3 button state
btn = Button()

# Connect motors
rightMotor = LargeMotor(OUTPUT_A)
assert rightMotor.connected, "Error: Right motor not connected"
leftMotor  = LargeMotor(OUTPUT_D)
assert leftMotor.connected, "Error: Left motor not connected"
ultraMotor = MediumMotor(OUTPUT_C)
assert ultraMotor.connected, "Error: Ultra motor not connected"

# Connect sensors
# https://sites.google.com/site/ev3python/learn_ev3_python/using-sensors
# https://media.readthedocs.org/pdf/ev3dev-lang/latest/ev3dev-lang.pdf

# Connect colour sensor
colorSensor = ColorSensor()
assert colorSensor.connected, "Error: Color sensor not connected"
colorSensor.mode = "COL-COLOR"

# Connect gyro sensor
gyroSensor = GyroSensor()
assert gyroSensor.connected, "Error: Color sensor not connected"
gyroSensor.mode = "GYRO-RATE" # Calibrate gyro
gyroSensor.mode = "GYRO-ANG"

# Connect ultrasonic sensor
ultraSensor = UltrasonicSensor()
assert ultraSensor.connected, "Error: Ultrasonic sensor not connected"
ultraSensor.mode = "US-DIST-CM" # Actually in millimetres

rightMotor.reset() # Set the current angle to 0
leftMotor.reset()
ultraMotor.reset()
rightMotor.stop_action = "brake"
leftMotor.stop_action = "brake"
ultraMotor.stop_action = "brake"

debug = False

# <MOVEMENT_MODULE>

# drive function constants
DRIVE_SPEED = 500
REVOLUTION = 360
DIAMETER = 5.6 # In centimetres
PI = 3.14
# turn function constants
'''
!!! UPDATE VALUES BELOW !!!
'''
TURN_SPEED = 100
TURN_OFFSET = 15 # In degrees
FINE_SPEED = 25
FINE_OFFSET = 2 # In degrees

# Drive forward a given distance in centimetres
def drive(distance):
    angle = distance * REVOLUTION / (PI * DIAMETER)
    rightMotor.position_sp += angle
    leftMotor.position_sp += angle
    rightMotor.run_to_abs_pos(speed_sp = DRIVE_SPEED)
    leftMotor.run_to_abs_pos(speed_sp = DRIVE_SPEED)
    leftMotor.wait_until_not_moving()

# Turn through a given angle
def turn(angle):
    '''
    !!! MISSING TURN FUNCTION !!!
    Changed OFFSET to TURN_OFFSET
    '''
    brake()
    leftMotor.wait_until_not_moving()
    rightMotor.position_sp = 0 # Reset position for drive function
    leftMotor.position_sp = 0

# Stop large motors
def brake():
    rightMotor.stop()
    leftMotor.stop()

# </MOVEMENT_MODULE>

# <SCANNING_MODULE>

SCAN_SPEED = 360

# Obtain values for front, left and right walls
def scan():
    ultraMotor.position_sp = 0
    ultraMotor.run_to_abs_pos(speed_sp = SCAN_SPEED)
    ultraMotor.wait_while('running')
    front = ultraSensor.value() // 42
    ultraMotor.position_sp = -90
    ultraMotor.run_to_abs_pos(speed_sp = SCAN_SPEED)
    ultraMotor.wait_while('running')
    left = ultraSensor.value() // 42
    ultraMotor.position_sp = 90
    ultraMotor.run_to_abs_pos(speed_sp = SCAN_SPEED)
    ultraMotor.wait_while('running')
    right = ultraSensor.value() // 42
    ultraMotor.position_sp = 0
    ultraMotor.run_to_abs_pos(speed_sp = SCAN_SPEED)
    print("Front: %d" % front)
    print("Left : %d" % left)
    print("Right: %d" % right)
    if (debug == False):
        updateWalls(front, left, right)

# </SCANNING_MODULE>

# <FINDING_MODULE>

MAZE_SIZE = 21
WALL = [' ',' ',' ','─',' ','┘','└','┴',' ','┐','┌','┬','│','┤','├','┼']

class Node:
    def __init__(self, x, y, review=False):
    	self.x = x
    	self.y = y
    	self.review = review

class Robot:
    def __init__(self, x, y, dir):
        self.x = x
        self.y = y
        self.dir = dir # 0 = North, 1 = East, 2 = South, 3 = West

robot = Robot(MAZE_SIZE // 2, MAZE_SIZE // 2, 0)

def mazePos(x, y):
    return (x + 1) + (MAZE_SIZE + 2) * (y + 1)

# Create wall array
# wallData = Array('i', range(MAZE_LENGTH))
wallData = [False] * (MAZE_SIZE+2) * (MAZE_SIZE+2)
for y in range(MAZE_SIZE):
    for x in range(MAZE_SIZE):
        if (y % 2 == 0 or (x % MAZE_SIZE) % 2 == 0):
            wallData[mazePos(x, y)] = True

# Create stack
pathStack = [Node(robot.x, robot.y)]

def mazePrint():
    for y in range(MAZE_SIZE):
        for x in range(MAZE_SIZE):
            i = mazePos(x, y)
            if (wallData[i] == True):
                wallIndex = 0
                if (wallData[i-1] == True):
                    wallIndex += 1;
                if (wallData[i+1] == True):
                    wallIndex += 2;
                if (wallData[i-(MAZE_SIZE+2)] == True):
                    wallIndex += 4;
                if (wallData[i+(MAZE_SIZE+2)] == True):
                    wallIndex += 8;
                print("%s" % WALL[wallIndex], end='')
            else:
                if (x == robot.x and y == robot.y):
                    print("#", end='')
                else:
                    print(" ", end='')
        print("")

def updateWalls(front, left, right):
    dirOffset = [-(MAZE_SIZE+2),1,(MAZE_SIZE+2),-1]
    remove = [0] * 4
    remove[currentDir] = front
    remove[(currentDir - 1) % 4] = left
    remove[(currentDir + 1) % 4] = right
    for dir in range(4):
        for i in range(remove[dir]):
            r = mazePos(robot.x, robot.y) + (2*i + 1) * dirOffset[dir]
            print("%i" % r)
            wallData[r] = False

# FIND ALGORITHM

def found():
    found = False
    while (found == False):
        scan()
        mazePrint()

# </FINDING_MODULE>

def debug():
    debug = True
    print("Entering debug mode...", end="\r")
    sleep(2)
    while True:
        try:
            print("0 | Debug drive function")
            print("1 | Debug turn function")
            print("2 | Debug scan function")
            value = int(input())
            if (value == 0):
                print("Drive: Input distance in centimetres")
                value = int(input())
                print("Driving %d centimetres..." % value, end="\r")
                drive(value)
            elif (value == 1):
                print("Turn: Input angle in degrees")
                value = int(input())
                print("Turning %d degrees..." % value, end="\r")
                turn(value)
            elif (value == 2):
                print("Scanning...", end="\r")
                scan()
            else:
                raise ValueError
        except ValueError:
            print("Error: Invalid input...", end="\r")
            sleep(2)

debug() # Enter debug mode

# Stop the motors before exiting.
rightMotor.stop()
leftMotor.stop()
ultraMotor.stop()
