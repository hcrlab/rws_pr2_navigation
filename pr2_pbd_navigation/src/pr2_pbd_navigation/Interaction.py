'''Main interaction loop'''
import roslib
import time

roslib.load_manifest('pr2_pbd_navigation')

from pr2_pbd_navigation.Robot import Robot


class Interaction:
    '''Finite state machine for the human interaction'''

    def __init__(self):
        self.robot = Robot.get_robot()

    def update(self):
        '''General update for the main loop'''
        time.sleep(0.1)
