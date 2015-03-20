"""Main interaction loop"""
import roslib
roslib.load_manifest('pr2_pbd_navigation')
import time
import rospy
from pr2_pbd_navigation.Session import Session
from pr2_pbd_navigation.Robot import Robot
from pr2_pbd_navigation.msg import NavigationCommand

class Interaction:
    """ Finite state machine for the human interaction """

    def __init__(self):
        self.robot = Robot.get_robot()
        self.session = Session.get_session()
        rospy.Subscriber('navigation_command', NavigationCommand, self.nav_command_cb)
        self.responses = {
            NavigationCommand.NEW_LOCATION: self.new_location,
            NavigationCommand.RECORD_LOCATION: self.record_location,
            NavigationCommand.SPIN_AROUND: self.spin_around,
            NavigationCommand.DELETE_CURRENT: self.delete_current_location,
            NavigationCommand.CHANGE_LOCATION_NAME: self.change_location_name,
            NavigationCommand.SWITCH_TO_LOCATION: self.switch_to_location,
            NavigationCommand.NAVIGATE_TO_CURRENT: self.navigate_to_current
        }
        rospy.loginfo('Interaction initialized.')

    def update(self):
        """ General update for the main loop """
        time.sleep(0.1)

    def nav_command_cb(self, command):
        """ Callback for when a command is received """
        if command.command in self.responses.keys():
            rospy.loginfo('Calling response for command ' + command.command)
            response_function = self.responses[command.command]
            response_function(command.param)
        else:
            rospy.logwarn('This command (' + command.command + ') is unknown.')

    def new_location(self, param):
        self.session.create_new_location()

    def record_location(self, param):
        self.session.record_current_location()

    def spin_around(self, param):
        self.robot.spin_base()

    def change_location_name(self, param):
        self.session.name_location(param)

    def delete_current_location(self, param):
        self.session.delete_current_location()

    def switch_to_location(self, param):
        self.session.switch_to_location_by_name(param)

    def navigate_to_current(self, param):
        self.robot.navigate_to(self.session.get_current_location())
