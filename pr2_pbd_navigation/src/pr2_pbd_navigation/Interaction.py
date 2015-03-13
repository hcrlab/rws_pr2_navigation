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
        rospy.Subscriber('gui_command', NavigationCommand, self.nav_command_cb)
        self.responses = {
            NavigationCommand.NEW_LOCATION: self.new_location,
            NavigationCommand.SAVE_LOCATION: self.save_location,
            NavigationCommand.SPIN_AROUND: self.spin_around,
            NavigationCommand.DELETE_CURRENT: self.delete_current_location,
            NavigationCommand.CHANGE_LOCATION_NAME: self.change_location_name
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

    def save_location(self, param):
        self.session.create_new_location()

    def spin_around(self, param):
        pass

    def change_location_name(self, param):
        self.session.name_location(param)

    def delete_current_location(self, param):
        self.session.delete_current_location()
