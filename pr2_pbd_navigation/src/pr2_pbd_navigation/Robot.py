#!/usr/bin/env python
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pr2_common_action_msgs.msg import TuckArmsAction, TuckArmsGoal
import roslib
import rospy
from tf import TransformListener
import tf
import time

roslib.load_manifest('pr2_pbd_navigation')


class Robot:
    """
    Controller for the robot: moves the base.
    """
    robot = None

    def __init__(self):
        self.tf_listener = TransformListener()

        self.nav_action_client = SimpleActionClient('move_base', MoveBaseAction)
        self.nav_action_client.wait_for_server()
        rospy.loginfo('Got response from move base action server.')
        self.tuck_arms_client = SimpleActionClient('tuck_arms', TuckArmsAction)
        self.tuck_arms_client.wait_for_server()
        rospy.loginfo('Got response from tuck arms action server.')

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

    def spin_base(self, rotate_count=2):
        """ Spin 360 * rotate_count degrees clockwise """
        rospy.loginfo("Orienting...")
        topic_name = '/base_controller/command'
        base_publisher = rospy.Publisher(topic_name, Twist)
        twist_msg = Twist()
        twist_msg.linear = Vector3(0.0, 0.0, 0.0)
        twist_msg.angular = Vector3(0.0, 0.0, 0.5)
        start_time = rospy.get_rostime()
        while rospy.get_rostime() < start_time + rospy.Duration(15.0 * rotate_count):
            base_publisher.publish(twist_msg)

    def navigate_to(self, location):
        if location is None:
            rospy.loginfo("No location provided, will not navigate.")
            return
        rospy.loginfo("Starting navigation.")
        base_pose = location.pose

        rospy.loginfo("Tucking arms for navigation.")
        goal = TuckArmsGoal()
        goal.tuck_left = True
        goal.tuck_right = True
        self.tuck_arms_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "/map"
        pose_stamped.pose = base_pose
        nav_goal = MoveBaseGoal()
        nav_goal.target_pose = pose_stamped
        # Client sends the goal to the Server
        self.nav_action_client.send_goal(nav_goal)
        elapsed_time = 0
        while (self.nav_action_client.get_state() == GoalStatus.ACTIVE
               or self.nav_action_client.get_state() == GoalStatus.PENDING):
            time.sleep(0.01)
            elapsed_time += 0.01
            if elapsed_time > 120:
                rospy.loginfo('Timeout waiting for base navigation to finish')
                self.nav_action_client.cancel_goal()
                break
        rospy.loginfo('Done with base navigation.')

        # Verify that base succeeded
        if (self.nav_action_client.get_state() != GoalStatus.SUCCEEDED):
            rospy.logwarn('Aborting because base failed to move to pose.')
            return False
        else:
            return True
