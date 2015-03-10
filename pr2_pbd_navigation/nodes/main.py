#!/usr/bin/env python
import roslib

from pr2_pbd_navigation.Interaction import Interaction

roslib.load_manifest('pr2_pbd_navigation')

import rospy


if __name__ == "__main__":
    global interaction
    rospy.init_node('pr2_pbd_navigation', anonymous=True)
    interaction = Interaction()
    #rospy.spin()
    while not rospy.is_shutdown():
        interaction.update()