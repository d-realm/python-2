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
ultraMotor = LargeMotor(OUTPUT_C)
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
assert gyroSensor.value() == 0, "Error: Reconnect color sensor"

# Connect ultrasonic sensor
ultraSensor = UltrasonicSensor()
assert ultraSensor.connected, "Error: Ultrasonic sensor not connected"
ultraSensor.mode = "US-DIST-CM" # Actually in millimetres

rightMotor.reset() # Set the current angle to 0
leftMotor.reset()
ultraMotor.reset()
rightMotor.stop_action = "brake"
leftMotor.stop_action = "brake"
ultraMotor.stop_action = "hold"

debug = False

# <MOVEMENT_MODULE>

# drive function constants
DRIVE_SPEED = 500
REVOLUTION = 360
DIAMETER = 5.6 # In centimetres
PI = 3.14
# turn function constants
TURN_SPEED = 750 # 0 to 1000
FINE_SPEED = 50
TURN_OFFSET = 50 # In degrees
FINE_OFFSET = 0

angleTarget = 0

# Drive forward a given distance in centimetres
def drive(distance):
    angle = distance * REVOLUTION / (PI * DIAMETER)
    rightMotor.position_sp += angle
    leftMotor.position_sp += angle
    rightMotor.run_to_abs_pos(speed_sp = DRIVE_SPEED)
    leftMotor.run_to_abs_pos(speed_sp = DRIVE_SPEED)
    while any(m.state for m in (leftMotor, rightMotor)):
        if (driftLeft == False and gyroSensor.value() < angleTarget - 1):
            leftMotor.run_to_abs_pos(speed_sp = 1.1*DRIVE_SPEED)
            driftLeft = True
            print("Drive: Fixing drift to the left...")
        elif (driftRight == False and angleTarget + 1 < gyroSensor.value()):
            rightMotor.run_to_abs_pos(speed_sp = 1.1*DRIVE_SPEED)
            driftRight = True
            print("Drive: Fixing drift to the right...")
        elif (driftLeft == True and gyroSensor.value() > angleTarget - 1):
            leftMotor.run_to_abs_pos(speed_sp = DRIVE_SPEED)
            driftLeft = False
            print("Drive: Fixed drift to the left!")
        elif (driftRight == False and angleTarget + 1 > gyroSensor.value()):
            rightMotor.run_to_abs_pos(speed_sp = DRIVE_SPEED)
            driftRight = False
            print("Drive: Fixed drift to the right!")

def driveForward():
    drive(42)
    if (robot.dir == 0):
        robot.y -= 2
    elif (robot.dir == 1):
        robot.x += 2
    elif (robot.dir == 2):
        robot.y += 2
    else:
        robot.x -= 2

# Turn through a given angle
def turn(angle):

    if (angle == 0):
        return
    angleTarget += angle
    direction = angle / abs(angle)

    current = gyroSensor.value()
    print("Turn: Current: %d" % current)
    print("Turn: Target: %d" % angleTarget)
    rightMotor.run_forever(speed_sp = -direction * TURN_SPEED)
    leftMotor.run_forever(speed_sp = direction * TURN_SPEED)

    while (current < angleTarget - TURN_OFFSET or angleTarget + TURN_OFFSET < current):
        current = gyroSensor.value()
        print("Turn: Fast: %d" % current)

    rightMotor.run_forever(speed_sp = -direction * FINE_SPEED)
    leftMotor.run_forever(speed_sp = direction * FINE_SPEED)

    while (current < angleTarget - FINE_OFFSET or angleTarget + FINE_OFFSET < current):
        current = gyroSensor.value()
        print("Turn: Slow: %d" % current)

    brake()
    leftMotor.wait_until_not_moving()
    rightMotor.position = 0 # Reset position for drive function
    leftMotor.position = 0

    if (angle == 90):
        robot.dir = (robot.dir + 1) % 4
    elif (angle == -90):
        robot.dir = (robot.dir + 3) % 4
    elif (angle == 180 or angle == -180):
        robot.dir = (robot.dir + 2) % 4

# Stop large motors
def brake():
    rightMotor.stop()
    leftMotor.stop()

# </MOVEMENT_MODULE>

# <SCANNING_MODULE>

SCAN_SPEED = 360

# Obtain values for front, left and right walls
directions = []
def scan():
    global directions
    ultraMotor.position_sp = 0
    ultraMotor.run_to_abs_pos(speed_sp = SCAN_SPEED)
    ultraMotor.wait_while('running')
    sleep(0.1)
    front = ultraSensor.value() // 420
    print("Scan: Front: %d" % ultraSensor.value())
    ultraMotor.position_sp = -90
    ultraMotor.run_to_abs_pos(speed_sp = SCAN_SPEED)
    ultraMotor.wait_while('running')
    sleep(0.1)
    right = ultraSensor.value() // 420
    print("Scan: Right: %d" % ultraSensor.value())
    ultraMotor.position_sp = 90
    ultraMotor.run_to_abs_pos(speed_sp = SCAN_SPEED)
    ultraMotor.wait_while('running')
    sleep(0.1)
    left = ultraSensor.value() // 420
    print("Scan: Left: %d" % ultraSensor.value())
    ultraMotor.position_sp = 0
    ultraMotor.run_to_abs_pos(speed_sp = SCAN_SPEED)
    print("Scan: Front: %d" % front)
    print("Scan: Right: %d" % right)
    print("Scan: Left : %d" % left)
    directions = [front, left, right]
    updateWalls(front, left, right)

# </SCANNING_MODULE>

# <FINDING_MODULE>

MAZE_SIZE = 31
WALL = [' ',' ',' ','#',' ','#','#','#',' ','#','#','#','#','#','#','#']
# WALL = [' ',' ',' ','─',' ','┘','└','┴',' ','┐','┌','┬','│','┤','├','┼']

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
wallData = []
def resetMaze():
    global wallData
    wallData = [False] * (MAZE_SIZE+2) * (MAZE_SIZE+2)
    for y in range(MAZE_SIZE):
        for x in range(MAZE_SIZE):
            if (y % 2 == 0 or (x % MAZE_SIZE) % 2 == 0):
                wallData[mazePos(x, y)] = True
resetMaze()

# Create stack
pathStack = [Node(robot.x, robot.y)]

def mazePrint():
    print("Printing maze...")
    for y in range(MAZE_SIZE):
        for x in range(MAZE_SIZE):
            i = mazePos(x, y)
            if (wallData[i] == True):
                wallIndex = 0
                if (wallData[i-1] == True):
                    wallIndex += 1
                if (wallData[i+1] == True):
                    wallIndex += 2
                if (wallData[i-(MAZE_SIZE+2)] == True):
                    wallIndex += 4
                if (wallData[i+(MAZE_SIZE+2)] == True):
                    wallIndex += 8
                print("%s" % WALL[wallIndex], end='')
            else:
                if (x == robot.x and y == robot.y):
                    print("O", end='')
                else:
                    print(" ", end='')
        print("")

def updateWalls(front, left, right):
    dirOffset = [-(MAZE_SIZE+2),1,(MAZE_SIZE+2),-1]
    remove = [0] * 4
    remove[robot.dir] = front
    remove[(robot.dir + 3) % 4] = left
    remove[(robot.dir + 1) % 4] = right
    for dir in range(4):
        if (remove[dir] > 4):
            remove[dir] == 4 # Fix potential index out of range error
        for i in range(remove[dir]):
            r = mazePos(robot.x, robot.y) + (2*i + 1) * dirOffset[dir]
            wallData[r] = False

# FIND ALGORITHM

turnDir = 1
found = False
def found():
    global turnDir
    global found
    while (found == False):
        scan()
        print("Available directions:")
        if (directions[0] > 0):
            print("Forward ", end='')
        if (directions[1] > 0):
            print("Left ", end'')
        if (directions[2] > 0):
            print("Right ", end='')
        print("")
        if (directions[0] > 0):
            turnDir *= -1
        elif (directions[1] > 0 and directions[2] > 0):
            turn(90 * turnDir)
            turnDir *= -1
        elif (directions[1] > 0):
            turn(-90)
        elif (directions[2] > 0):
            turn(90)
        else:
            turn(180)
        driveForward()

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
            print("3 | Debug mazePrint function")
            print("4 | Debug updateWalls function")
            print("5 | Reset maze")
            print("9 | Print robot data")
            value = int(input())
            if (value == 0):
                print("Driving 42 centimetres..." % value, end="\r")
                driveForward()
            elif (value == 1):
                print("Turn: Input angle in degrees")
                value = int(input())
                print("Turning %d degrees..." % value, end="\r")
                turn(value)
            elif (value == 2):
                print("Scanning...", end="\r")
                scan()
            elif (value == 3):
                mazePrint()
            elif (value == 4):
                print("Update: Input walls in front")
                front = int(input())
                print("Update: Input walls to left")
                left = int(input())
                print("Update: Input walls to right")
                right = int(input())
                updateWalls(front, left, right)
            elif (value == 5):
                resetMaze()
            elif (value == 9):
                print("x: %i, y = %i, dir = %i" % (robot.x, robot.y, robot.dir))
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
