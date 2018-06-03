#!/usr/bin/env python
import rospy
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
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
import roslib
roslib.load_manifest('poi_navigator')
import actionlib

from poi_navigator.msg import GoToPoiAction, GoToPoiGoal


if __name__ == '__main__':
    rospy.init_node('poi_navigator_client')

    # Before you run poi_navigator_client.py, use "roslaunch turtlebot_rviz_launchers view_navigation.launch"
    # to give the turtlebot an initial pose estimate.

    client = actionlib.SimpleActionClient('poi_navigator_server', GoToPoiAction)
    rospy.loginfo("wait_for_server")
    client.wait_for_server()

    goal = GoToPoiGoal()
    goal.destination_name = 'hall2_adjacent_rm212_collaboration_room'

    rospy.loginfo("send_goal")
    client.send_goal(goal)
    rospy.loginfo("wait_for_result")
    client.wait_for_result()
