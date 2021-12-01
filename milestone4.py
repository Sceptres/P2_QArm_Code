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

def move_effector(arm: qarm, pos: list) -> None:
    '''
    Input:
        arm: The Q-Arm instance
        pos: An array with the requested position. Format: [X, Y, Z]

    Moves the arm end effector to pos
    '''
    arm.move_arm(pos[0], pos[1], pos[2])
    time.sleep(1)


def control_gripper(arm: qarm, should_close: bool) -> None:
    '''
    Input:
        arm: The QArm instance
        should_close: A bool representing whether the gripper should open/close

    Opens/Closes the gripper
    '''
    
    arm.control_gripper(40 if should_close else -40)


def is_at_pos(arm: qarm, pos: list):
    '''
    Input:
        arm: The QArm instance
        pos: The position to check

    Return: True -> If the arm is at the position pos
            False -> If the arm is not at the position pos
    '''

    return list(arm.effector_position()) == pos


def is_small(cage_id: int) -> bool:
    '''
    Input:
        arm: The QArm instance
        cage_id: The id of the cage

    Return: True -> If the cage has an id in the range [1, 3], ie small size
            False otherwise
    '''
    return cage_id >= 1 and cage_id <= 3


def is_large(cage_id: int) -> bool:
    '''
    Input:
        arm: The QArm instance
        cage_id: The id of the cage

    Return: True -> If the cage has an id in the range [4, 6]; ie large size
            False otherwise
    '''
    
    return cage_id >= 4 and cage_id <= 6


def is_valid_id(cage_id: int) -> bool:
    '''
    Input:
        cage_id: The id of the cage

    Return: True -> If the id is valid; ie in the range [1, 6]
            False otherwise
    '''

    return cage_id <= 6 and cage_id >= 1


def generate_cage_id(collected_cages: list) -> int:
    '''
    Input:
        collected_cages: The collected cages

    Return: The id of the new cage
    '''
    
    possible_cages = [1, 2, 3, 4, 5, 6]

    # Remove collected cages
    for i in collected_cages:
        possible_cages.remove(i)

    return choice(possible_cages) if not (len(possible_cages) == 0) else -1


def get_pickup_pos(cage_id: int) -> list:

    '''
    Input:
        cage_id: The id of the cage

    Return: the position to pickup the cage.
            Depends on the size of the cage.
                 
    '''

    size = None

    if is_small(cage_id): # Is the cage small?
        size = "Small"
    elif is_large(cage_id): # Is the cage large?
        size = "Large"
    
    switcher = {
            "Small": [0.453, 0.0, 0.036],   # Small cage pickup position
            "Large": [0.52, 0, 0.037]       # Large cage pickup position
        }
    
    return switcher.get(size, get_home_pos())

def get_home_pos() -> list:
    '''
    Return: The home position of the arm
    '''
    return [0.406, 0, 0.483]

def get_autoclave_pos(cage_id: int) -> list:
    '''
    Input:
        cage_id: The id of the cage

    Return: The appropriate drop off position 
    '''

    switcher = {
        1: [-0.565, 0.224, 0.331],  # Red Small, cage_id = 1
        2: [0.0, -0.606, 0.343],    # Green Small, cage_id = 2
        3: [0.0, 0.606, 0.343],     # Blue Small, cage_id = 3
        4: [-0.392, 0.159, 0.248],  # Red Large, cage_id = 4
        5: [0, -0.427, 0.248],      # Green Large, cage_id = 5
        6: [0, 0.418, 0.248],       # Blue Large, cage_id = 6
        }

    return switcher.get(cage_id, get_home_pos())


def control_autoclave_bin(arm: qarm, cage_id: int, should_open: bool) -> None:
    '''
    Input:
        arm: The QArm instance
        cage_id: The id of the cage
        should_open: A boolean representing autoclave state
                     True -> Open
                     False -> Close

    Opens/Closes the appropriate autoclave according to should_open
    '''
    
    # Quit if small cage
    if is_small(cage_id): return None 
    else:
        # Get appropriate autoclave
        autoclave = get_autoclave(arm, cage_id)

        # Open/Close autoclave
        for _ in range(2):
            autoclave.open_drawer(should_open)
            time.sleep(1)


def get_autoclave(arm: qarm, cage_id: int) -> autoclave_sim:
    '''
    Input:
        arm: QArm instance
        cage_id: The id of the cage

    Return: The instance of the autoclave of the cage
    '''

    if not is_valid_id(cage_id):
        return None
    
    elif cage_id % 3 == 1: # Is the cage red?
        return arm.red_autoclave

    elif cage_id % 3 == 2: # Is the cage Green?
        return arm.green_autoclave

    elif cage_id % 3 == 0: # Is the cage Blue?
        return arm.blue_autoclave


def face_autoclave(arm: qarm, cage_id: int) -> None:
    '''
    Input:
        arm: QArm instance
        cage_id: The id of the cage

    Moves the effector to face the autoclave
    '''

    # Set facing direction
    pos = get_autoclave_pos(cage_id)
    pos[2] = 0.5

    # Face the autoclave
    move_effector(arm, pos)


def handle_open_autoclave(arm: qarm, is_autoclave_open: bool, cage_id: int) -> bool:
    '''
    Input:
        arm: The QArm instance
        is_autoclave_open: Current status of autoclave bin
        cage_id: The id of the cage

    Opens/Closes the autoclave according to its status

    Return: New autoclave status
            True -> If autoclave has been opened
            False -> If autoclave has been closed
    '''
    if is_small(cage_id): return is_autoclave_open
    
    # Action to take according to current autoclave status
    should_open = not is_autoclave_open

    # Open/Close autoclave accordingly
    control_autoclave_bin(arm, cage_id, should_open)

    # Return new autoclave state
    return should_open


def handle_move_effector(arm: qarm, is_autoclave_open: bool, has_cage: bool, was_cage_delivered: bool, cage_id: int) -> None:
    '''
    Input:
        arm: The QArm instance
        is_autoclave_open: Status of autoclave bin
        has_cage: Status of cage aquisition
        was_cage_delivered: Status of the cage delivery
        cage_id: The id of the cage

    Moves the arm effector to the appropriate position to pickup/drop off the cage
    according to its status
    '''

    if was_cage_delivered and not is_autoclave_open: # Was the cage delivered?
        face_autoclave(arm, cage_id)
        time.sleep(1)
        move_effector(arm, get_home_pos())

    elif not has_cage and not was_cage_delivered: # Is the cage not collected yet?
        # Move to pickup
        move_effector(arm, get_pickup_pos(cage_id)) 
        
    elif not is_at_pos(arm, get_autoclave_pos(cage_id)): # Was the cage collected?
        
        if is_large(cage_id) and not is_autoclave_open: return None
        
        # Move to drop off
        move_effector(arm, get_home_pos())
        time.sleep(1)
        face_autoclave(arm, cage_id)
        time.sleep(1)
        move_effector(arm, get_autoclave_pos(cage_id))


def handle_gripper(arm: qarm, has_cage: bool) -> bool:
    '''
    Input:
        arm: The QArm instance
        has_cage: Status of the cage acquisition

    Return: The new status of cage acquisition
    '''
    # Open/Close accordingly
    cage_status = not has_cage
    control_gripper(arm, cage_status)

    return cage_status
    

def handle_input(arm: qarm) -> None:
    '''
    Input:
        arm: The QArm instance

    Handles EMG input
    '''

    collected_cages = []            # The collected cages throughout the program
    
    is_autoclave_open: bool = False # Autoclave bin status
    has_cage: bool = False          # Cage acquisition status
    was_cage_delivered = False      # Cage delivery status

    # Spawn first cage
    cage_id: int = generate_cage_id(collected_cages)
    arm.spawn_cage(cage_id)

    # Perform until all 6 cages have been processed
    while cage_id != -1:

        # Update cade delivery status
        was_cage_delivered = is_at_pos(arm, get_autoclave_pos(cage_id)) and not has_cage
        
        if arm.emg_right() < 1 and arm.emg_left() < 1: # Is the user idle
            time.sleep(1.5)
            
        elif arm.emg_right() >= 1 and arm.emg_left() >= 1: # Should autoclave open/close
            is_autoclave_open = handle_open_autoclave(arm, is_autoclave_open, cage_id)
            time.sleep(2)
            
        elif arm.emg_right() >= 1 and arm.emg_left() < 1: # Should the effector move to position
            handle_move_effector(arm, is_autoclave_open, has_cage, was_cage_delivered, cage_id)
            time.sleep(2)
            
        elif arm.emg_left() >= 1 and arm.emg_right() < 1: # Should the gripper hold/release the cage
            has_cage = handle_gripper(arm, has_cage)
            time.sleep(2)

        if was_cage_delivered: # Was the cage delivered
            # Record newly processed cages 
            if cage_id not in collected_cages: collected_cages.append(cage_id)

            # Spawn the new cage when arm is back at home
            if is_at_pos(arm, get_home_pos()):
                cage_id = generate_cage_id(collected_cages)
                if is_valid_id(cage_id): arm.spawn_cage(cage_id)

                was_cage_delivered = False


def run(arm: qarm):
    '''
    Input:
        arm: The QArm instance

    Runs the program
    '''

    arm.home()
    handle_input(arm)
