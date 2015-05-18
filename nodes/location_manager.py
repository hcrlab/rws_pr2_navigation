#!/usr/bin/env python
import os
import yaml
import rospy

import roslib
from rws_pr2_navigation.Robot import Robot
from rws_pr2_navigation.Session import Session

roslib.load_manifest('rws_pr2_navigation')
from rws_pr2_navigation.srv import GetSavedLocations, ExecuteLocationStepResponse, ExecuteLocationStep
from rws_pr2_navigation.srv import GetSavedLocationsResponse


def get_saved_locations(dummy):
    locations = Session.get_saved_locations()
    return GetSavedLocationsResponse(locations)

def execute_location_step(req):
    rospy.loginfo('Executing location step.')
    step_id = req.step_id
    data_directory = rospy.get_param('/rws_pr2_navigation/locationsRoot')
    file_extension = ".yaml"
    file_path = data_directory + str(step_id) + file_extension
    if not os.path.exists(file_path):
        return ExecuteLocationStepResponse(0)
    with open(file_path, 'r') as content_file:
        pose = yaml.load(content_file).pose
        rospy.loginfo('Sending goal.')
        result = robot.navigate_to(pose)
        status = False
        if result:
            status = True
        return ExecuteLocationStepResponse(status)



if __name__ == '__main__':
    robot = Robot.get_robot()
    rospy.init_node('location_manager')
    s1 = rospy.Service('get_saved_locations', GetSavedLocations, get_saved_locations)
    s2 = rospy.Service('execute_location_step', ExecuteLocationStep, execute_location_step)
    rospy.spin()
