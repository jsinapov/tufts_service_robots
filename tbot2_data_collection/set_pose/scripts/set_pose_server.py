#!/usr/bin/env python
from __future__ import print_function

import rospy
import roslib
import rospkg
import yaml
import os
import actionlib
import math

roslib.load_manifest('set_pose')
from set_pose.msg import SetPoseAction
from set_pose.msg import SetPoseActionGoal
from set_pose.msg import SetPoseActionResult
from set_pose.msg import SetPoseActionFeedback
from set_pose.msg import SetPoseGoal
from set_pose.msg import SetPoseResult
from set_pose.msg import SetPoseFeedback

from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from std_msgs.msg import String
from std_msgs.msg import Header
from poi_name_locator.srv import PoiNameLocator
from poi_name_locator.srv import PoiNameLocatorRequest
from poi_name_locator.srv import PoiNameLocatorResponse
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler


class SetPose:
    def __init__(self):
        self.current_pose = None  # type: PoseWithCovarianceStamped
        self.new_pose = None  # type: PoseWithCovarianceStamped
        self.amcl_pose_subscriber = None  # type: rospy.topics.Subscriber
        self.initialpose_publisher = None  # type: rospy.topics.Publisher
        self.initialpose_publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)

        # I don't know a general way to do this. Based on my observations, it seems like 2 should work.
        self.position_tolerance = 2

        self.server = actionlib.SimpleActionServer('set_pose_server', SetPoseAction, self.execute, False)
        self.server.start()
        rospy.loginfo('Ready to accept SetPoseActionGoal messages on set_pose_server actionlib server')

    def execute(self, goal):  # type: (SetPoseActionGoal) -> None
        self.new_pose = goal.pose
        self.new_pose.header.stamp = rospy.Time.now()

        rospy.loginfo("Setting new pose:\n{}".format(self.new_pose))
        self.initialpose_publisher.publish(self.new_pose)

        self.amcl_pose_subscriber = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.handle_amcl_pose_msg)

        # handle_amcl_pose_msg will unsubscribe, and set self.amcl_pose_subscriber = None when it's done.
        loop_count = 0
        while (not rospy.core.is_shutdown()) and self.amcl_pose_subscriber is not None:
            rospy.rostime.wallsleep(0.1)
            loop_count += 1
            if loop_count > 100:  # 100 * 0.1 sec = 10 sec
                rospy.logwarn("amcl_pose_subscriber is taking a long time. Am I stuck?")

        self.server.set_succeeded()

    def handle_amcl_pose_msg(self, pose):  # type: (PoseWithCovarianceStamped) -> None
        if pose is None:
            rospy.logwarn("Ignoring pose is None")
            return
        if not hasattr(pose, 'pose'):
            rospy.logwarn("Ignoring pose does not have attribute 'pose'")
            return
        if pose.pose is None:
            rospy.logwarn("Ignoring pose.pose is None")
            return
        if not hasattr(pose.pose, 'pose'):
            rospy.logwarn("Ignoring pose.pose does not have attribute 'pose'")
            return
        if pose.pose.pose is None:
            rospy.logwarn("Ignoring pose.pose.pose is None")
            return
        if not hasattr(pose.pose.pose, 'position'):
            rospy.logwarn("Ignoring pose.pose.pose does not have attribute 'position'")
            return
        if pose.pose.pose.position is None:
            rospy.logwarn("Ignoring pose.pose.pose.position is None")
            return
        if not hasattr(pose.pose.pose.position, 'x'):
            rospy.logwarn("Ignoring pose.pose.pose.position does not have attribute 'x'")
            return
        if not hasattr(pose.pose.pose.position, 'y'):
            rospy.logwarn("Ignoring pose.pose.pose.position does not have attribute 'y'")
            return
        if pose.pose.pose.position.x is None:
            rospy.logwarn("Ignoring pose.pose.pose.position.x is None")
            return
        if pose.pose.pose.position.y is None:
            rospy.logwarn("Ignoring pose.pose.pose.position.y is None")
            return
        if math.isnan(pose.pose.pose.position.x):
            rospy.logwarn("Ignoring math.isnan(pose.pose.pose.position.x)")
            return
        if math.isnan(pose.pose.pose.position.y):
            rospy.logwarn("Ignoring math.isnan(pose.pose.pose.position.y)")
            return

        current_position = pose.pose.pose.position

        self.current_pose = pose
        desired_position = self.new_pose.pose.pose.position
        rospy.loginfo('Got current_pose: x, y = {}, {}'.format(current_position.x, current_position.y))

        if current_position.x > desired_position.x - self.position_tolerance and \
                current_position.x < desired_position.x + self.position_tolerance and \
                current_position.y > desired_position.y - self.position_tolerance and \
                current_position.y < desired_position.y + self.position_tolerance:
            # We have reached our desired position
            self.amcl_pose_subscriber.unregister()
            self.amcl_pose_subscriber = None
            return

        rospy.loginfo('Publishing desired position: x, y = {}, {}'.format(desired_position.x, desired_position.y))
        self.new_pose.header.seq = self.current_pose.header.seq + 1
        self.new_pose.header.stamp = rospy.Time.now()
        self.initialpose_publisher.publish(self.new_pose)


if __name__ == '__main__':
    rospy.init_node('set_pose_server')
    server = SetPose()
    rospy.spin()
