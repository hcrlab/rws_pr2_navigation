#!/usr/bin/env python
import roslib

from rws_pr2_navigation.Interaction import Interaction

roslib.load_manifest('rws_pr2_navigation')

import rospy


if __name__ == "__main__":
    global interaction
    rospy.init_node('rws_pr2_navigation', anonymous=True)
    interaction = Interaction()
    #rospy.spin()
    while not rospy.is_shutdown():
        interaction.update()