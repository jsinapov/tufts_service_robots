#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
import yaml
import rospkg
import os
import actionlib
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler
import roslib
roslib.load_manifest('poi_scan')
import actionlib

from poi_name_locator.srv import PoiNameLocator
from poi_name_locator.srv import PoiNameLocatorRequest
from poi_name_locator.srv import PoiNameLocatorResponse

from poi_scan.msg import PoiScanAction, PoiScanGoal


def initialize():
    rospy.loginfo("Waiting for poi_name_locator service")
    rospy.wait_for_service('poi_name_locator')
    poi_name_locate = rospy.ServiceProxy('poi_name_locator', PoiNameLocator)
    request = PoiNameLocatorRequest('hall2_adjacent_rm212_collaboration_room')
    response = poi_name_locate(request)  # type: PoiNameLocatorResponse
    # response cannot be None. If server tries to return None, the line above will raise rospy.ServiceException
    position = response.position  # type: Point

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
    pose.position = position

    client.send_goal(goal)
    client.wait_for_result()

    # TODO: Should probably check the result.
    result = client.get_result()

    return result


def scan():
    client = actionlib.SimpleActionClient('poi_scan_server', PoiScanAction)
    client.wait_for_server()

    goal = PoiScanGoal()
    goal.topics = ['/camera/depth/image_raw', '/camera/rgb/image_raw', '/amcl_pose', '/people_tracker_measurements', 
          '/audio', 'sensor_msgs/PointCloud' ]
    # goal.topics = ['odom', 'clock']  # available in gazebo
    goal.bagfile_name_prefix = 'first_tests'
    goal.num_stops = 8
    goal.duration = 3.0  # sec
    goal.tune_rotation = 1.33
    goal.return_to_original = True
    goal.upload_url = 'https://turtlecloud.eecs.tufts.edu/api/v1.0/rosbags/'
    goal.upload_token = '' # DO NOT GIT COMMIT, EDIT ON TURTLEBOT ITSELf

    client.send_goal(goal)
    client.wait_for_result()


def main():
    rospy.init_node('poi_scan_client')

    # Before you begin poi_scan_client, be sure to give the turtlebot an initial pose estimate.

    # Before sending a goal to poi_scan_server, be sure the turtlebot is navigating itself, so its sensory inputs
    # don't conflict with the position it thinks it's in. This is important so the turtlebot doesn't get confused
    # about its own location and orientation while it spins around and perceives its surroundings.
    #
    # In this simple example, I start by placing the turtlebot somewhere near the collaboration room, and
    # using "roslaunch turtlebot_rviz_launchers view_navigation.launch" to set the initial pose estimate, and then
    # my example initialize() function is hard-coded to navigate to hall2_adjacent_rm212_collaboration_room
    # before doing the poi_scan_server procedure in the scan() function.

    # The initialize() function just drives to the specified location
    # initialize()

    # The scan() function is how we actually do the scan.
    scan()


if __name__ == '__main__':
    main()
