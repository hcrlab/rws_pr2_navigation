import rospy

import roslib
from rws_pr2_navigation.Session import Session

roslib.load_manifest('rws_pr2_navigation')
from rws_pr2_navigation.srv import GetSavedLocations
from rws_pr2_navigation.srv import GetSavedLocationsResponse


def get_saved_locations():
    locations = Session.get_saved_locations()
    return GetSavedLocationsResponse(locations)


if __name__ == '__main__':
    rospy.init_node('location_manager')
    s1 = rospy.Service('get_saved_locations', GetSavedLocations, get_saved_locations)
    # s2 = rospy.Service('execute_location_step', ExecuteLocationStep, execute_location_step)
