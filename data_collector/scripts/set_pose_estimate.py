#!/usr/bin/env python
from __future__ import print_function

import rospy
import rospkg
import yaml
import os
import actionlib
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


class SetPoseEstimate:
    def __init__(self):
        self.current_pose = None  # type: PoseWithCovarianceStamped
        self.new_pose = None  # type: PoseWithCovarianceStamped
        self.amcl_pose_subscriber = None  # type: rospy.topics.Subscriber
        self.initialpose_publisher = None  # type: rospy.topics.Publisher

        # I don't know a general way to do this. Based on my observations, it seems like 2 should work.
        self.position_tolerance = 2

    def handle_amcl_pose_msg(self, pose):
        self.current_pose = pose
        current_position = pose.pose.pose.position
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

    def run(self):
        rospy.init_node('set_initial_pose')

        self.new_pose = PoseWithCovarianceStamped()
        self.new_pose.header.frame_id = "map"
        self.new_pose.pose.pose.position.x = -4.26896037359
        self.new_pose.pose.pose.position.y = 2.0599534457
        self.new_pose.pose.pose.orientation.w = 1
        self.new_pose.pose.covariance = [0.22257526179165055, -0.002681367573648785, 0.0, 0.0, 0.0, 0.0,
                                    -0.002681367573648785, 0.20214520026529215, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.06791405198597536]

        self.initialpose_publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)

        self.amcl_pose_subscriber = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.handle_amcl_pose_msg)

        # handle_amcl_pose_msg will unsubscribe, and set self.amcl_pose_subscriber = None when it's done.
        loop_count = 0
        while (not rospy.core.is_shutdown()) and self.amcl_pose_subscriber is not None:
            rospy.rostime.wallsleep(0.1)
            loop_count += 1
            if loop_count > 100:  # 100 * 0.1 sec = 10 sec
                rospy.logwarn("amcl_pose_subscriber is taking a long time. Am I stuck?")


if __name__ == "__main__":
    a = SetPoseEstimate()
    a.run()
