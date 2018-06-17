#!/usr/bin/env python
from __future__ import print_function

import rospy
import rospkg
import roslib
import yaml
import os
import actionlib
import time
import math
from move_base_msgs.msg import MoveBaseAction
# from move_base_msgs.msg import MoveBaseActionClient
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

roslib.load_manifest('poi_scan')
from poi_scan.msg import PoiScanAction, PoiScanGoal

roslib.load_manifest('set_pose')
from set_pose.msg import SetPoseAction
from set_pose.msg import SetPoseActionGoal
from set_pose.msg import SetPoseActionResult
from set_pose.msg import SetPoseActionFeedback
from set_pose.msg import SetPoseGoal
from set_pose.msg import SetPoseResult
from set_pose.msg import SetPoseFeedback


class TraversalConfig:
    def __init__(self, yaml_load):  # type: (dict) -> None
        if 'start_point' in yaml_load.keys():
            self.start_point = yaml_load['start_point']  # type: str
        else:
            self.start_point = None  # type: str

        if 'force_capture' in yaml_load.keys():
            self.force_capture = yaml_load['force_capture']  # type: list
        else:
            self.force_capture = []  # type: list

        self.waypoints = yaml_load['waypoints']  # type: list
        self.sample_distance = float(yaml_load['sample_distance'])  # type: float  # Also, cast ints and strs to float.
        self.existing_data = yaml_load['existing_data']  # type: list  # list of dicts, each dict contains 'x' and 'y'
        self.topics = yaml_load['topics']  # type: list
        self.output_dir = yaml_load['output_dir']  # type: str
        self.poi_scan_num_stops = yaml_load['poi_scan_num_stops']  # type: int
        self.poi_scan_duration = yaml_load['poi_scan_duration']  # type: float
        self.poi_scan_return_to_original_orientation = \
            yaml_load['poi_scan_return_to_original_orientation']  # type: bool


class TraverseMapByWaypoints:
    def __init__(self):
        self.config = None  # type: TraversalConfig
        self.current_pose = None  # type: PoseWithCovarianceStamped
        self.poi_name_locate = None  # type: callable(PoiNameLocatorRequest)
        self.amcl_pose_subscriber = None  # type: rospy.Subscriber
        self.move_base_action_client = None  # the type is not clear. type() returns "instance" ... so ... ok ...

        self.is_interruptible = False

        # I don't know a general way to do this. Based on my observations, it seems like 2 should work.
        self.position_tolerance = 2

    def setup(self):
        rospy.init_node('traverse_map_by_waypoints')

        # Read config from yaml file
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('data_collector')
        traversal_config_yml_path = os.path.join(pkg_path, 'share', 'traversal_config.yaml')

        with open(traversal_config_yml_path, "r") as f:
            self.config = TraversalConfig(yaml.load(f.read()))

        # Create output dir if not exists
        if not os.path.isdir(self.config.output_dir):
            os.makedirs(self.config.output_dir)

        # Setup poi_name_locator service
        rospy.loginfo("Waiting for poi_name_locator service")
        rospy.wait_for_service('poi_name_locator')
        self.poi_name_locate = rospy.ServiceProxy('poi_name_locator', PoiNameLocator)

        # Subscribe to pose updates. It might take a few seconds before we get the first one.
        rospy.loginfo("Creating amcl_pose_subscriber")
        self.amcl_pose_subscriber = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.handle_amcl_pose_msg)

        # if self.config.start_point is None, the user must set initialpose manually via
        # "roslaunch turtlebot_rviz_launchers view_navigation.launch" before running traverse_map_by_waypoints.py
        # if self.config.start_point is not None, we set initialpose now:
        if self.config.start_point is not None:
            rospy.loginfo("Looking up start_point")
            request = PoiNameLocatorRequest(self.config.start_point)
            response = self.poi_name_locate(request)  # type: PoiNameLocatorResponse
            # response cannot be None. If server tries to return None, the line above will raise rospy.ServiceException
            position = response.position  # type: Point

            new_pose = PoseWithCovarianceStamped()
            new_pose.header.frame_id = "map"
            new_pose.pose.pose.position = position

            # hard-coded orientation of yaw = 0
            new_pose.pose.pose.orientation.w = 1

            # hard-coded covariance I got by sampling it once.
            new_pose.pose.covariance = [0.22257526179165055, -0.002681367573648785, 0.0, 0.0, 0.0, 0.0,
                                             -0.002681367573648785, 0.20214520026529215, 0.0, 0.0, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                             0.0, 0.0,
                                             0.0, 0.0, 0.0, 0.06791405198597536]

            rospy.loginfo("Waiting for set_pose_server")
            client = actionlib.SimpleActionClient('set_pose_server', SetPoseAction)
            client.wait_for_server()
            goal = SetPoseGoal()
            goal.pose = new_pose
            client.send_goal(goal)
            client.wait_for_result()  # it can take several seconds for set_pose to complete

        rospy.loginfo("Creating move_base_action_client")
        self.move_base_action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_action_client.wait_for_server()

        # Above, I subscribed to pose updates, and may have set initialpose, both of which may take several seconds,
        # and can run in parallel. Now wait until I know for sure I've got at least one:
        loop_count = 0
        while (not rospy.core.is_shutdown()) and self.current_pose is None:
            rospy.rostime.wallsleep(0.1)
            loop_count += 1
            if loop_count > 50:
                rospy.logwarn("amcl_pose_subscriber is taking a long time. Am I stuck?")

    def handle_amcl_pose_msg(self, pose):  # type: (PoseWithCovarianceStamped) -> None
        self.current_pose = pose
        if self.is_interruptible and self.need_to_collect_data():
            self.move_base_action_client.cancel_all_goals()

    def get_point_from_waypoint(self, waypoint):  # type: (str) -> Point
        request = PoiNameLocatorRequest(waypoint)
        response = self.poi_name_locate(request)  # type: PoiNameLocatorResponse
        # response cannot be None. If server tries to return None, the line above will raise rospy.ServiceException
        position = response.position  # type: Point
        return position

    def collect_data(self):
        position = self.current_pose.pose.pose.position
        self.config.existing_data.append({'x': position.x, 'y': position.y})
        rospy.loginfo('existing_data: {}'.format(self.config.existing_data))

        client = actionlib.SimpleActionClient('poi_scan_server', PoiScanAction)
        client.wait_for_server()

        goal = PoiScanGoal()
        goal.topics = self.config.topics

        # bagfile_name_prefix will be like "/foo/bar/1528572992" which is timestamp in seconds.
        # poi_scan_server generates suffixes like "_pos0.bag" etc.
        goal.bagfile_name_prefix = os.path.join(self.config.output_dir, "{}".format(int(time.time())))

        goal.num_stops = self.config.poi_scan_num_stops
        goal.duration = self.config.poi_scan_duration
        goal.return_to_original = self.config.poi_scan_return_to_original_orientation

        client.send_goal(goal)
        client.wait_for_result()

        existing_data_path = os.path.join(self.config.output_dir, "existing_data.txt")
        with open(existing_data_path, 'w') as f:
            f.write('{}'.format(self.config.existing_data))

    def drive_to_waypoint(self, waypoint):  # type: (str) -> None
        waypoint_pos = self.get_point_from_waypoint(waypoint)  # type: Point

        rospy.loginfo('drive_to_waypoint({}): Starting...'.format(waypoint))

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

        self.move_base_action_client.send_goal(goal)
        self.move_base_action_client.wait_for_result()

        # Pointless. It just returns the final pose, which we already have in self.current_pose
        # result = self.move_base_action_client.get_result()

        if self.arrived_at_waypoint(waypoint):
            rospy.loginfo('drive_to_waypoint({}) Completed.')
        else:
            rospy.loginfo('drive_to_waypoint({}) Interrupted.')

    def need_to_collect_data(self):  # type: () -> bool
        for xy_dict in self.config.existing_data:  # type: dict
            x = xy_dict['x']  # type: float
            y = xy_dict['y']  # type: float
            cur_pos = self.current_pose.pose.pose.position  # type: Point
            # If either x or y is too far away, then we know the datapoint is too far away, so we can skip
            # doing the actual distance calculation
            if abs(cur_pos.x - x) > self.config.sample_distance:
                continue
            if abs(cur_pos.y - y) > self.config.sample_distance:
                continue
            if (math.sqrt(
                (cur_pos.x - x) ** 2 + (cur_pos.y - y) ** 2
            ) > self.config.sample_distance):
                continue
            # If we get here, we found a datapoint whose distance is < sample_distance away, so we don't
            # need to collect data.
            return False
        # If we get here, we checked all datapoints and didn't find any close enough, so we need to collect data.
        rospy.loginfo("Triggered: Need to collect data.")
        return True

    def arrived_at_waypoint(self, waypoint):  # type: (str) -> bool
        desired_point = self.get_point_from_waypoint(waypoint)
        actual_point = self.current_pose.pose.pose.position  # type: Point
        if abs(desired_point.x - actual_point.x) > self.position_tolerance:
            return False
        if abs(desired_point.y - actual_point.y) > self.position_tolerance:
            return False

        # I could do a euclidian distance calculation here, but who cares. The whole concept of position_tolerance
        # is subjective and non-ideal, until someday when we might learn to get a more accurate pass/fail result
        # from move_base.

        return True

    def traverse_map_by_waypoints(self):
        self.setup()

        self.drive_to_waypoint(self.config.waypoints[0])

        if self.need_to_collect_data():
            self.collect_data()

        waypoint_num = 1
        while waypoint_num < len(self.config.waypoints):
            self.is_interruptible = True
            self.drive_to_waypoint(self.config.waypoints[waypoint_num])  # could interrupt by need_to_collect_data()
            self.is_interruptible = False

            if self.need_to_collect_data():
                self.collect_data()

            if self.arrived_at_waypoint(self.config.waypoints[waypoint_num]):
                if self.config.waypoints[waypoint_num] in self.config.force_capture:
                    self.collect_data()
                waypoint_num += 1
                continue


if __name__ == "__main__":
    tm = TraverseMapByWaypoints()
    tm.traverse_map_by_waypoints()
