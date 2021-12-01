'''
Authors:
    1. Abdallah Alqashqish
    2. Brian Zhang
    3. Jaskeerat Sandhu
'''

import time
import sys
from random import randint
from random import choice
sys.path.append('../')

from Common_Libraries.p2_sim_lib import *

import os
from Common_Libraries.repeating_timer_lib import repeating_timer

from functions import run

def update_sim ():
    try:
        arm.ping()
    except Exception as error_update_sim:
        print (error_update_sim)

arm = qarm()
update_thread = repeating_timer(2, update_sim)

## *******************************************
## DO NOT EDIT ANY OF THE CODE ABOVE THIS LINE
## *******************************************

'''
cage_id is a number that represents the cage.
    1 -> Small Red
    2 -> Small Green
    3 -> Small Blue
    4 -> Large Red
    5 -> Large Green
    6 -> Large Blue
'''

'''
How to control the QArm
1. Flex only the right arm to move the effector
2. Flex only the left arm to control the gripper
3. Flex both to control the autoclave bins (Only works if the cage is large)

Program will not allow user to move to bin before opening the bin
Program will not allow user to move away from bin before closing the bin
'''
run(arm)
