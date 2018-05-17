#!/usr/bin/python3

from time import sleep
import sys, os

from ev3dev.ev3 import *

# Will need to check EV3 button state
btn = Button()

# Connect motors
rightMotor = LargeMotor(OUTPUT_D)
assert rightMotor.connected, "Error: Right motor not connected"
leftMotor  = LargeMotor(OUTPUT_A)
assert leftMotor.connected, "Error: Left motor not connected"
ultraMotor = LargeMotor(OUTPUT_B)
assert ultraMotor.connected, "Error: Ultra motor not connected"
clawMotor = MediumMotor(OUTPUT_C)
assert clawMotor.connected, "Error: Claw motor not connected"

# Connect sensors
# https://sites.google.com/site/ev3python/learn_ev3_python/using-sensors
# https://media.readthedocs.org/pdf/ev3dev-lang/latest/ev3dev-lang.pdf

# Connect colour sensor
colorSensor = ColorSensor()
assert colorSensor.connected, "Error: Color sensor not connected"
colorSensor.mode = "COL-COLOR"

# Connect gyro sensor
gyroSensor = GyroSensor()
assert gyroSensor.connected, "Error: Gyro sensor not connected"
gyroSensor.mode = "GYRO-RATE" # Calibrate gyro
gyroSensor.mode = "GYRO-ANG"
sleep(1)
assert gyroSensor.value() == 0, "Error: Gyro sensor uncalibrated"

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
FINE_OFFSET = 1

angleTarget = 0

# Drive forward a given distance in centimetres
def drive(distance, speed):
    angle = distance * REVOLUTION / (PI * DIAMETER)
    rightMotor.position_sp += angle
    leftMotor.position_sp += angle
    rightMotor.run_to_abs_pos(speed_sp = speed)
    leftMotor.run_to_abs_pos(speed_sp = speed)
    rightMotor.wait_while("running")
    leftMotor.wait_while("running")

def driveForward():
    drive(42, DRIVE_SPEED)
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
    global angleTarget
    angleTarget += angle
    direction = angle / abs(angle)

    current = gyroValue()
    print("Turn: Current: %d" % current)
    print("Turn: Target: %d" % angleTarget)
    rightMotor.run_forever(speed_sp = -direction * TURN_SPEED)
    leftMotor.run_forever(speed_sp = direction * TURN_SPEED)

    while (current < angleTarget - TURN_OFFSET or angleTarget + TURN_OFFSET < current):
        current = gyroValue()
        # print("Turn: Fast: %d" % current)

    rightMotor.run_forever(speed_sp = -direction * FINE_SPEED)
    leftMotor.run_forever(speed_sp = direction * FINE_SPEED)

    while (current < angleTarget - FINE_OFFSET or angleTarget + FINE_OFFSET < current):
        current = gyroValue()
        # print("Turn: Slow: %d" % current)

    brake()

    rightMotor.position_sp = 0
    rightMotor.position = 0 # Reset position for drive function
    leftMotor.position_sp = 0
    leftMotor.position = 0

    if (angle % 360 == 90):
        robot.dir = (robot.dir + 1) % 4
    elif (angle % 360 == 270):
        robot.dir = (robot.dir + 3) % 4
    elif (angle % 360 == 180):
        robot.dir = (robot.dir + 2) % 4

# Stop large motors
def brake(hold=False):
    rightMotor.stop()
    leftMotor.stop()

# </MOVEMENT_MODULE>

# <SCANNING_MODULE>

SCAN_SPEED = 360

# Obtain values for front, left and right walls
distanceScan = [0, 0, 0]
directions = [0, 0, 0, 0]
def scan():
    global distanceScan
    global directions
    if (ultraMotor.position != 0):
        ultraMotor.position_sp = 0
        ultraMotor.run_to_abs_pos(speed_sp = SCAN_SPEED)
        ultraMotor.wait_while('running')
        sleep(0.1)
    front = ultraSensor.value() // 420
    distanceScan[0] = ultraSensor.value()
    print("Scan: Front: %d" % ultraSensor.value())
    ultraMotor.position_sp = -95
    ultraMotor.run_to_abs_pos(speed_sp = SCAN_SPEED)
    ultraMotor.wait_while('running')
    sleep(0.1)
    left = ultraSensor.value() // 420
    distanceScan[1] = ultraSensor.value()
    print("Scan: Left: %d" % ultraSensor.value())
    ultraMotor.position_sp = 95
    ultraMotor.run_to_abs_pos(speed_sp = SCAN_SPEED)
    ultraMotor.wait_while('running')
    sleep(0.1)
    right = ultraSensor.value() // 420
    distanceScan[2] = ultraSensor.value()
    print("Scan: Right: %d" % ultraSensor.value())
    ultraMotor.position_sp = 0
    ultraMotor.run_to_abs_pos(speed_sp = SCAN_SPEED)
    ultraMotor.wait_while('running')
    ultraMotor.stop()
    print("Scan: Front: %d" % front)
    print("Scan: Left : %d" % left)
    print("Scan: Right: %d" % right)
    directions = [front, right, 1, left]
    updateWalls(front, left, right)

# </SCANNING_MODULE>

# <FINDING_MODULE>

MAZE_SIZE = 31
WALL = [' ',' ',' ','#',' ','#','#','#',' ','#','#','#','#','#','#','#']
# WALL = [' ',' ',' ','─',' ','┘','└','┴',' ','┐','┌','┬','│','┤','├','┼']

class Node: # REMEMBER TO COPY THIS IN
    def __init__(self, x, y, deadEnd=True):
    	self.x = x
    	self.y = y
    	self.deadEnd = deadEnd
    	self.directions = []
    	self.reference = 0

    def __str__(self):
        return "x: %d, y: %d, deadEnd: %d" % (self.x, self.y, self.deadEnd)

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
# pathStack = [Node(robot.x, robot.y)]
pathStack = []

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

dirOffset = [-(MAZE_SIZE+2),1,(MAZE_SIZE+2),-1]
def updateWalls(front, left, right):
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

ALIGN_DISTANCE = 100
def alignForward():
    current = ultraSensor.value()
    print("Align: Driving %d centimetres" % ((current - ALIGN_DISTANCE) // 10))
    if (current < ALIGN_DISTANCE - 10 or ALIGN_DISTANCE + 10 < current):
        drive((current - ALIGN_DISTANCE) // 10, DRIVE_SPEED // 5)

# FIND ALGORITHM

def scanRed():
    turn(-20)
    turn(40)
    turn(-20)

def checkRed():
    colors = ("unknown", "black", "blue", "green", "yellow", "red", "white", "brown")
    redFound = False
    if (colorSensor.value() == 5):
        redFound = True
    return redFound

found = False
def foundVictim():
    global found
    found = True
    print("Found: Victim found")
    Sound.tone([(1000, 500, 500)] * 3)

turnDir = 1
def find():
    global turnDir
    global found
    global pathStack
    while (found == False):
        # scanRed()
        # mazePrint()
        scan()
        if (directions[0] == 0):
            alignForward()
        if (distanceScan[1] < 100):
            print("Find: Aligning with left wall")
            turn(-90)
            alignForward()
            turn(90)
        elif (distanceScan[2] < 100):
            print("Find: Aligning with right wall")
            turn(90)
            alignForward()
            turn(-90)

        node = nodeExists(robot.x, robot.y, 4)
        if (node == False):
            print("Find: Making node at (%d, %d)" % (robot.x, robot.y))
            node = Node(robot.x, robot.y)
            node.directions = list(directions)
            node.reference = robot.dir
            pathStack.append(node)

        print("Find: Available directions (robot):")
        if (node.directions[dirRef(1, node.reference, robot.dir)] > 0):
            turnDir = 90
            print("Right ", end='')
        if (node.directions[dirRef(3, node.reference, robot.dir)] > 0):
            turnDir = 270
            print("Left ", end='')
        if (node.directions[dirRef(0, node.reference, robot.dir)] > 0):
            turnDir = 0
            print("Forward ", end='')

        print("")
        print("Find: Available directions (wallData):")
        possCount = 0;
        if (wallData[mazePos(robot.x, robot.y) + dirOffset[robot.dir]] == False):
            print("Forward ", end='')
            possCount += 1
        if (wallData[mazePos(robot.x, robot.y) + dirOffset[(robot.dir + 3) % 4]] == False):
            print("Left ", end='')
            possCount += 1
        if (wallData[mazePos(robot.x, robot.y) + dirOffset[(robot.dir + 1) % 4]] == False):
            print("Right ", end='')
            possCount += 1
        print("")
        deadEnd = False
        if (possCount <= 1 and deadEnd == True):
            print("Find: Still dead end")
            node.deadEnd = True
        elif (possCount == 0):
            print("Find: Dead end")
            node.deadEnd = True
            deadEnd = True
            turnDir = 180
        else:
            deadEnd = False
            if (node.directions[dirRef(0, node.reference, robot.dir)] > 0):
                node.directions[dirRef(0, node.reference, robot.dir)] = 0
                turnDir = 0
                print("Find: Driving forward")
            elif (node.directions[dirRef(3, node.reference, robot.dir)] > 0):
                node.directions[dirRef(3, node.reference, robot.dir)] = 0
                turnDir = 270
                print("Find: Turning left")
            elif (node.directions[dirRef(1, node.reference, robot.dir)] > 0):
                node.directions[dirRef(1, node.reference, robot.dir)] = 0
                turnDir = 90
                print("Find: Turning right")

        if (found == False):
            turn(turnDir)
            driveForward()
            claw("close")
            if (checkRed() == True):
                foundVictim()
                claw("close more")
                return
            else:
                claw("open", False)

def nodeExists(x, y, dir):
    global pathStack
    offset = [[0,-1], [1,0], [0,1], [-1,0], [0,0]]

    for node in pathStack:
        if (node.x == x + offset[dir][0] and (node.y == y + offset[dir][1])):
            return node
    return False

def dirRef(dir, fromRef, toRef):
    return (dir + (4 - fromRef) + toRef) % 4

# </FINDING_MODULE>

WHEEL_90 = 220
def gyroCalibrate():
    global gyroOffClock
    rightMotor.position_sp = -WHEEL_90
    leftMotor.position_sp = WHEEL_90
    rightMotor.run_to_abs_pos(speed_sp = TURN_SPEED, stop_action = "hold")
    leftMotor.run_to_abs_pos(speed_sp = TURN_SPEED, stop_action = "hold")
    rightMotor.wait_while('running')
    leftMotor.wait_while('running')
    sleep(0.5)
    gyroOffClock = gyroSensor.value()
    print("Gyro value: %d" % gyroOffClock)
    gyroSensor.mode = "GYRO-RATE" # Calibrate gyro
    gyroSensor.mode = "GYRO-ANG"
    sleep(0.5)
    print("Gyro should be at 0: %d" % gyroSensor.value())
    rightMotor.reset()
    leftMotor.reset()

gyroOffClock = 90
def gyroValue():
    return int(gyroSensor.value() * (90 / gyroOffClock))

def pickUpCan():
    if (checkRed() == True):
        claw("close")

# Function to open or close the claw
def claw(command, wait=True):
    print("..........................................")
    if (command == "open"):
        print("Opening Claw")
        clawMotor.position_sp += -1200
        clawMotor.run_to_abs_pos(speed_sp = 1000)
    if (command == "close"):
        print("Closing claw")
        clawMotor.position_sp += 1200
        clawMotor.run_to_abs_pos(speed_sp = 1000)
    if (command == "stop"):
        print("Stopping claw")
        clawMotor.stop()
        return
    if (command == "close more"):
        clawMotor.position_sp += 300
        clawMotor.run_to_abs_pos(speed_sp = 1000)
    if (command == "snip snap"):
        claw("close")
        claw("open")

    if (wait == True):
        clawMotor.wait_while("running")

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
            print("7 | Run FIND")
            print("9 | Print robot data")
            value = int(input())
            if (value == 0):
                print("Driving 42 centimetres...", end="\r")
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
            elif (value == 6):
                print("0 | Open claw")
                print("1 | Close claw")
                print("2 | Stop claw")
                print("3 | Snip snap")
                print("4 | Close more")
                x = int(input())
                if (x == 0):
                    claw("open")
                elif (x == 1):
                    claw("close")
                elif (x == 2):
                    claw("stop")
                elif (x == 3):
                    claw("snip snap")
                elif (x == 4):
                    claw("close more")
            elif (value == 7):
                find()
            elif (value == 9):
                print("x: %i, y = %i, dir = %i" % (robot.x, robot.y, robot.dir))
            else:
                raise ValueError
        except ValueError:
            print("Error: Invalid input...", end="\r")
            sleep(2)

# gyroCalibrate()
claw("open")
find()
debug() # Enter debug mode

# Stop the motors before exiting.
rightMotor.stop()
leftMotor.stop()
ultraMotor.stop()
