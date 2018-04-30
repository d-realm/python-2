#!/usr/bin/python3

from time import sleep

MAZE_SIZE = 21
MAZE_LENGTH = MAZE_SIZE * MAZE_SIZE

class Node:
    def __init__(self, x, y, review=False):
    	self.x = x
    	self.y = y
    	self.review = review

stack = [Node(MAZE_SIZE // 2, MAZE_SIZE // 2)]

stack.append(Node(MAZE_SIZE // 2 + 1, MAZE_SIZE // 2))
stack.append(Node(MAZE_SIZE // 2 + 1, MAZE_SIZE // 2 + 1, True))

current = stack.pop()
print("x: %i" % current.x)
print("y: %i" % current.y)
if (current.review == False):
    print("false │")
else:
    print("true │")
