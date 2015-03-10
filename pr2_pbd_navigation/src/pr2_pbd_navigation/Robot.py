#!/usr/bin/env python
import roslib

roslib.load_manifest('pr2_pbd_navigation')

class Robot:
    '''
    Controller for the robot: moves the base.
    '''
    robot = None

    def __init__(self):
        pass

    @staticmethod
    def get_robot():
        if Robot.robot is None:
            Robot.robot = Robot()
        return Robot.robot

