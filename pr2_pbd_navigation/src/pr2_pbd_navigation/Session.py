"""Everything related to an experiment session"""
from geometry_msgs.msg import Pose
import roslib
roslib.load_manifest('pr2_pbd_navigation')
from functools import partial
import os
from os import listdir
from os.path import isfile, join
import yaml

import rospy

from pr2_pbd_navigation.msg import NavSystemState
from pr2_pbd_navigation.srv import GetNavSystemState
from pr2_pbd_navigation.srv import GetNavSystemStateResponse
from pr2_pbd_navigation.msg import Location
from pr2_pbd_navigation.Robot import Robot


class Session:
    """This class holds and maintains the list of locations"""

    session = None
    data_directory = rospy.get_param('/pr2_pbd_navigation/dataRoot', '/home/sonyaa/pbd_location_saved/')
    file_extension = ".yaml"

    def __init__(self):
        self.selected_location = -1

        if not os.path.exists(Session.data_directory):
            os.makedirs(Session.data_directory)
        self.locations = self.get_saved_locations()
        self.current_location_index = 0 if len(self.locations) > 0 else None
        if self.current_location_index is not None:
            self.initialize_viz()

        self._state_publisher = rospy.Publisher('nav_system_state',
                                                NavSystemState)
        rospy.Service('get_nav_system_state', GetNavSystemState,
                      self.get_nav_system_state_cb)

        self._update_state()


    @staticmethod
    def get_session():
        if Session.session is None:
            Session.session = Session()
        return Session.session

    def get_nav_system_state_cb(self, dummy):
        """ Response to the experiment state service call"""
        return GetNavSystemStateResponse(self._get_nav_system_state())

    def _update_state(self):
        """ Publishes a message with the latest state"""
        state = self._get_nav_system_state()
        self._state_publisher.publish(state)

    def _get_nav_system_state(self):
        """ Creates a message with the latest state"""
        return NavSystemState(
            map(lambda act: act.name, self.locations),
            map(lambda act: act.id, self.locations),
            -1 if self.current_location_index is None else self.current_location_index,
            Pose() if self.current_location_index is None else self.get_current_location().pose)

    def create_new_location(self):
        """ Creates new location """
        if self.n_locations() > 0:
            self.reset_viz()
        new_loc = Location(name="Unnamed " + str(len(self.locations)), id=len(self.locations))
        Session.record_and_store_location(new_loc)
        self.locations.append(new_loc)
        self.current_location_index = len(self.locations) - 1
        self._update_state()

    def n_locations(self):
        """ Returns the number of locations saved so far """
        return len(self.locations)

    def get_current_location(self):
        """ Returns the current location """
        if self.current_location_index is None:
            return None
        return self.locations[self.current_location_index]

    def delete_current_location(self):
        """ Removes the current location """
        if self.current_location_index is None:
            rospy.logwarn('No location is selected.')
            return
        if self.n_locations() > 0:
            Session.delete_location_file(self.get_current_location())
            self.locations.pop(self.current_location_index)
            self.current_location_index = None
        else:
            rospy.logwarn('No locations saved yet.')
        self._update_state()

    def record_current_location(self):
        """ Saves location of the robot into the current location msg """
        if self.current_location_index is None:
            rospy.logwarn('No location is selected.')
            return
        if self.n_locations() > 0:
            Session.record_and_store_location(self.locations[self.current_location_index])
        else:
            rospy.logwarn('No locations created yet.')
        self._update_state()

    def set_pose(self, pose):
        """ Saves the given pose into the current location msg """
        if self.current_location_index is None:
            rospy.logwarn('No location is selected.')
            return
        if self.n_locations() > 0:
            self.locations[self.current_location_index].pose = pose
            Session.store_location(self.locations[self.current_location_index])
        else:
            rospy.logwarn('No locations created yet.')
        self._update_state()

    def get_location_name(self, location_number):
        if self.n_locations() > 0 and 0 <= location_number < self.n_locations():
            location = self.locations[location_number]
            if location.name is not None:
                return location.name
        return None

    def switch_to_location(self, location_number):
        """ Switches to indicated location """
        if self.n_locations() > 0:
            if location_number < self.n_locations() and location_number >= 0:
                self.reset_viz()
                self.current_location_index = location_number
                self.initialize_viz()
                success = True
            else:
                rospy.logwarn('Cannot switch to location '
                              + str(location_number))
                success = False
        else:
            rospy.logwarn('No locations created yet.')
            success = False
        self._update_state()
        return success

    def switch_to_location_by_name(self, location_name):
        return self.switch_to_location(next((i for i, loc in enumerate(self.locations)
                                             if loc.name == location_name), -1))

    def name_location(self, new_name):
        if self.current_location_index is None:
            rospy.logwarn('No location is selected.')
            return
        if len(self.locations) > 0:
            self.locations[self.current_location_index].name = new_name
            Session.store_location(self.locations[self.current_location_index])
            self._update_state()

    @staticmethod
    def record_and_store_location(location):
        """ Saves location of the robot into the current location msg and stores the location msg on disk"""
        location.pose = Robot.get_robot().get_base_pose()
        Session.store_location(location)

    @staticmethod
    def store_location(location):
        """ Saves location to file """
        if location.id is None:
            location.id = 0
            while os.path.isfile(Session.get_file(location.id)):
                location.id += 1
        loc_file = open(Session.get_file(location.id), 'w')
        loc_file.write(Session.to_string(location))
        loc_file.close()

    @staticmethod
    def delete_location_file(location):
        """ Deletes the file of the location """
        os.remove(Session.get_file(location.id))

    @staticmethod
    def to_string(location):
        """ Gets the yaml representing this location """
        return yaml.dump(location)

    @staticmethod
    def load(loc_f_id):
        if type(loc_f_id) is int:
            file_path = Session.get_file(loc_f_id)
        else:
            file_path = loc_f_id
        loc_file = open(file_path, 'r')
        loc = Session.from_string(loc_file)
        loc_file.close()
        return loc

    @staticmethod
    def from_string(yaml_str):
        return yaml.load(yaml_str)

    @staticmethod
    def get_file(location_id):
        return Session.data_directory + str(location_id) + Session.file_extension

    @staticmethod
    def get_saved_locations():
        locations = map(Session.load,
                        filter(lambda f: f.endswith(Session.file_extension),
                               filter(isfile,
                                      map(partial(join, Session.data_directory),
                                          listdir(Session.data_directory)))))
        locations.sort(key=lambda location: location.id)
        return locations

    def reset_viz(self):
        #TODO
        pass

    def initialize_viz(self):
        #TODO
        pass
