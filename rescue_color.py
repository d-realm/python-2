#!/usr/bin/python3

from time import sleep
from threading import Thread
from multiprocessing import Process, Value
import sys, os

from ev3dev.ev3 import *

# Will need to check EV3 button state
btn = Button()

colorSensor = ColorSensor()
assert colorSensor.connected, "Error: Color sensor not connected"
colorSensor.mode = "COL-COLOR"

def check_red():
    colors = ("unknown", "black", "blue", "green", "yellow", "red", "white", "brown")
    red_found = False
    if (colorSensor.value() == 5):
        red_found = True

    return red_found

def found_victim():
    print("victim found")
    Sound.speak("Beep Beep Beep").wait()

while not (btn.any()):
    if (check_red() == True):
        found_victim()
        break;
    sleep(0.02)

sys.exit()
