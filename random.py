# Generates a random integer 0-3
import sys, os
from time import sleep

def random_int(possible_ints):
    possible_int = False
    while (possible_int == False):
        random_byte = os.urandom(1)
        random_int = random_byte[0] % 4
        i = 0
        while (i < len(possible_ints)):
            if (random_int == possible_ints[i]):
                possible_int = True
            i += 1
    return random_int

while (True):
    array = [1, 3]
    print("%d" % random_int(array))
    sleep(0.2)
