#!/usr/bin/python3

from time import sleep
from multiprocessing import Process, Value, Array

def mazePos(x, y):
    return (x + 1) + (MAZE_SIZE + 2) * (y + 1)

MAZE_SIZE = 19

class Node:
    def __init__(self, x, y, review=False):
    	self.x = x
    	self.y = y
    	self.review = review

currentPos = Node(MAZE_SIZE // 2, MAZE_SIZE // 2)
currentDir = 1

print("%d %d" % (currentPos.x, currentPos.y))

mazeWall = [False] * (MAZE_SIZE+2) * (MAZE_SIZE+2)

def updateWalls(front, left, right):
    dirOffset = [-(MAZE_SIZE+2),1,(MAZE_SIZE+2),-1]
    remove = [0] * 4
    remove[currentDir] = front
    remove[(currentDir - 1) % 4] = left
    remove[(currentDir + 1) % 4] = right
    for dir in range(4):
        for i in range(remove[dir]):
            r = mazePos(currentPos.x, currentPos.y)+(2*i+1)*dirOffset[dir]
            print("%i" % r)
            mazeWall[r] = False

for y in range(MAZE_SIZE):
    for x in range(MAZE_SIZE):
        if (y % 2 == 0 or (x % MAZE_SIZE) % 2 == 0):
            mazeWall[mazePos(x, y)] = True

updateWalls(3,2,0)

WALL = [' ',' ',' ','─',' ','┘','└','┴',' ','┐','┌','┬','│','┤','├','┼']
for y in range(MAZE_SIZE):
    for x in range(MAZE_SIZE):
        i = mazePos(x, y)
        if (mazeWall[i] == True):
            wallIndex = 0
            if (mazeWall[i-1] == True):
                wallIndex += 1;
            if (mazeWall[i+1] == True):
                wallIndex += 2;
            if (mazeWall[i-MAZE_SIZE-2] == True):
                wallIndex += 4;
            if (mazeWall[i+MAZE_SIZE+2] == True):
                wallIndex += 8;
            print("%s" % WALL[wallIndex], end='')
        else:
            if (x == currentPos.x and y == currentPos.y):
                print("#", end='')
            else:
                print(" ", end='')
    print("")

# Initialise the turret process, independent from the main program
def init_turret(delay):
    # Begin a new process which will run concurrently
    p = Process(target=scan_turret, args=(delay, mazeWall))
    p.daemon = True
    p.start() # Initialise process

def scan_turret(delay, array):
    sleep(1)
    print("hello")
    for i in range(0, MAZE_LENGTH):
        array[i] = 1
        print("%d" % array[i])

'''
init_turret(1)

for i in range(0, MAZE_LENGTH):
    print("%d" % mazeWall[i])

sleep(2)
'''
