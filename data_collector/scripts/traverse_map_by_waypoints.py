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
from std_msgs.msg import String
from std_msgs.msg import Header
from poi_name_locator.srv import PoiNameLocator
from poi_name_locator.srv import PoiNameLocatorRequest
from poi_name_locator.srv import PoiNameLocatorResponse
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler


class TraversalConfig:
    def __init__(self, yaml_load):
        self.start_point = yaml_load['start_point']  # type: str
        self.waypoints = yaml_load['waypoints']  # type: list
        self.sample_distance = float(yaml_load['sample_distance'])  # type: float  # Also, cast ints and strs to float.
        self.existing_data = yaml_load['existing_data']  # type: list  # list of dicts, each dict contains 'x' and 'y'


class TraverseMapByWaypoints:
    def __init__(self):
        self.config = None  # type: TraversalConfig
        self.current_position = None  # type: Point
        self.poi_name_locate = None  # type: callable(PoiNameLocatorRequest)

    def setup(self):
        # Read config from yaml file
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('data_collector')
        traversal_config_yml_path = os.path.join(pkg_path, 'share', 'traversal_config.yaml')

        with open(traversal_config_yml_path, "r") as f:
            self.config = TraversalConfig(yaml.load(f.read()))

        # TODO: To set initialpose programmatically:
        # https://answers.ros.org/question/227129/localizing-turtlebot-programmatically-via-initialpose-topic/

        rospy.loginfo("Waiting for poi_name_locator service")
        rospy.wait_for_service('poi_name_locator')
        self.poi_name_locate = rospy.ServiceProxy('poi_name_locator', PoiNameLocator)

    def get_point_from_waypoint(self, waypoint):
        request = PoiNameLocatorRequest(waypoint)
        response = self.poi_name_locate(request)  # type: PoiNameLocatorResponse
        # response cannot be None. If server tries to return None, the line above will raise rospy.ServiceException
        position = response.position  # type: Point
        return position

    def collect_data(self):
        # TODO: implement
        raise RuntimeError()
        pass

    def drive_to_unconditional(self, waypoint):
        waypoint_pos = self.get_point_from_waypoint(waypoint)  # type: Point

        rospy.loginfo('drive_to_unconditional({}): Starting...'.format(waypoint))

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        target_pose = goal.target_pose  # type: PoseStamped

        header = target_pose.header  # type: Header
        header.frame_id = "/map"
        header.stamp = rospy.Time.now()

        pose = target_pose.pose  # type: Pose

        # pose.orientation  by default is just a bunch of 0's, which is not valid because the length of the
        # vector is 0. Length of vector must be 1, and for map navigation, z-axis must be vertical, so by setting
        # w = 1, it's the same as yaw = 0
        pose.orientation.w = 1
        pose.position = waypoint_pos

        client.send_goal(goal)
        client.wait_for_result()

        result = client.get_result()

        rospy.loginfo('drive_to_unconditional({}) Result: {}'.format(waypoint, result))

        return result

    def need_to_collect_data(self):
        # TODO: implement
        raise RuntimeError()
        pass

    def traverse_map_by_waypoints(self):
        self.setup()

        # TODO: The purpose for config.start_point is so you can put the turtlebot there and let the code (right here)
        # set the initial pose estimate from data. But for now I don't know how to do that. You have to set the initial
        # pose manually before launching traverse_map_by_waypoints.py, using
        # "roslaunch turtlebot_rviz_launchers view_navigation.launch" and we're actually ignoring config.start_point

        self.drive_to_unconditional(self.config.waypoints[0])

        self.collect_data()


if __name__ == "__main__":
    rospy.init_node('traverse_map_by_waypoints')
    tm = TraverseMapByWaypoints()
    tm.traverse_map_by_waypoints()
