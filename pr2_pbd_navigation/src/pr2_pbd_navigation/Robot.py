#!/usr/bin/env python
from geometry_msgs.msg import Pose, Point, Quaternion
import roslib
import rospy
from tf import TransformListener
import tf

roslib.load_manifest('pr2_pbd_navigation')


class Robot:
    """
    Controller for the robot: moves the base.
    """
    robot = None

    def __init__(self):
        self.tf_listener = TransformListener()

    @staticmethod
    def get_robot():
        if Robot.robot is None:
            Robot.robot = Robot()
        return Robot.robot

    def get_base_pose(self):
        try:
            ref_frame = "/map"
            time = self.tf_listener.getLatestCommonTime(ref_frame,
                                                        "/base_link")
            (position, orientation) = self.tf_listener.lookupTransform(
                ref_frame, "/base_link", time)
            base_pose = Pose()
            base_pose.position = Point(position[0], position[1], position[2])
            base_pose.orientation = Quaternion(orientation[0], orientation[1],
                                               orientation[2], orientation[3])
            rospy.loginfo('Current base pose:')
            rospy.loginfo(base_pose)
            return base_pose
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            rospy.logwarn('Something wrong with transform request for base state.')
            return None

