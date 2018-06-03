#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('poi_navigator')
from std_msgs.msg import String
from std_msgs.msg import Header
import yaml
from pprint import pprint
import rospkg
import os
import actionlib
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
import threading
import time
import subprocess
import signal
import math

from poi_navigator.msg import GoToPoiAction, GoToPoiGoal


class PoiNavigatorServer:
    def __init__(self):
        # self.poi is used like this:
        #   x = self.poi['elevator_adjacent_foyer']['x']
        #   y = self.poi['elevator_adjacent_foyer']['y']
        self.poi = None  # type: dict
        self.server = actionlib.SimpleActionServer('poi_navigator_server', GoToPoiAction, self.execute, False)

    def load(self):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('poi_navigator')
        locations_yml_path = os.path.join(pkg_path, 'share', 'locations.yaml')

        with open(locations_yml_path, "r") as f:
            self.poi = yaml.load(f.read())

        self.server.start()

    def execute(self, goal):
        if self.poi is None:
            rospy.logerr("Error: execute() called before load()")
            raise RuntimeError("execute() called before load()")

        destination_name = goal.destination_name  # type: str

        if destination_name not in self.poi.keys():
            rospy.logwarn('destination_name does not exist: {}'.format(destination_name))
            self.server.set_aborted(text='destination_name does not exist: {}'.format(destination_name))
            return

        rospy.loginfo('{} navigating to {}'.format(rospy.get_caller_id(), destination_name))

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        target_pose = goal.target_pose  # type: PoseStamped

        header = target_pose.header  # type: Header
        header.frame_id = "/map"
        header.stamp = rospy.Time.now()

        pose = target_pose.pose  # type: Pose

        # pose.orientation  by default is just a bunch of 0's, which is not valid because the length of the
        # vector is 0. Length of vector must be 1, and for map navigation, z-axis must be vertical, so:
        pose.orientation.w = 1

        pose.position.x = self.poi[destination_name]['x']
        pose.position.y = self.poi[destination_name]['y']

        rospy.logdebug('{} goal x,y: {},{}'.format(rospy.get_caller_id(), pose.position.x, pose.position.y))

        client.send_goal(goal)

        client.wait_for_result()

        return client.get_result()


if __name__ == '__main__':
    rospy.init_node('poi_navigator_server')
    poi_navigator_server = PoiNavigatorServer()
    poi_navigator_server.load()
    rospy.spin()
