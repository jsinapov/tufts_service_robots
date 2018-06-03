#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('poi_scan')
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
import psutil
import signal
import math

from poi_scan.msg import PoiScanAction, PoiScanGoal


class PoiScanServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('poi_scan_server', PoiScanAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        topics = goal.topics  # type: list
        bagfile = goal.bagfile  # type: str
        num_stops = goal.num_stops  # type: int
        stop_time = goal.stop_time  # type: float
        return_to_original = goal.return_to_original  # type: bool

        move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        move_base_client.wait_for_server()

        orig_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped).pose.pose  # type: Pose
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "/map"
        move_base_goal.target_pose.pose.position = orig_pose.position

        # Always start by facing a consistent direction
        yaw = 0  # radians counter-clockwise relative to map north
        q = quaternion_from_euler(0, 0, yaw)
        move_base_goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        move_base_client.send_goal(move_base_goal)
        move_base_client.wait_for_result()

        # TODO: I should probably check this.
        # move_base_result = move_base_client.get_result()

        # LZ4 is very fast compression, won't slow down your system, usually speeds up your system due to less I/O
        # BZ2 is very compute-intensive, strong, slow compression. Hence not using it here.
        # I found, that if you run 'rosbag' by shelling out like this, you're really just launching another python
        # script that will itself run subprocess. So I'm not changing, but it might be useful to know someday:
        # https://github.com/ros/ros_comm/blob/melodic-devel/tools/rosbag/src/rosbag/rosbag_main.py
        cmd = ['rosbag', 'record', '-O', bagfile, '--lz4']
        cmd.extend(topics)
        rospy.loginfo('Starting rosbag record: {}'.format(cmd))

        if not bagfile.startswith("/"):
            # I figure, if they specified an absolute path, they know exactly where the file is going.
            # If they are writing to CWD, it's wherever roscore is running, which is probably not what they expect.
            rospy.logwarn("Warning: Creating bagfile {}".format(os.path.join(os.getcwd(), bagfile)))

        # record in background
        bag_process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        rospy.loginfo('Pausing for arbitrary delay, to let rosbag start...')
        time.sleep(4.0)  # ummm... arbitrary delay, to let rosbag start... it's not instantaneous...

        rospy.loginfo("Position 0/{}".format(num_stops - 1))
        time.sleep(stop_time)

        for i in range(1, num_stops):
            yaw = i * 2 * math.pi / num_stops
            q = quaternion_from_euler(0, 0, yaw)
            move_base_goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

            move_base_goal.target_pose.header.stamp = rospy.Time.now()
            move_base_client.send_goal(move_base_goal)
            move_base_client.wait_for_result()

            rospy.loginfo("Position {}/{}".format(i, num_stops - 1))
            time.sleep(stop_time)

        # Finish rotating the whole circle by returning to yaw=0 position
        yaw = 0  # radians counter-clockwise relative to map north
        q = quaternion_from_euler(0, 0, yaw)
        move_base_goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        move_base_client.send_goal(move_base_goal)
        move_base_client.wait_for_result()

        rospy.loginfo("Completed full circle. Sending SIGINT to PID {}".format(bag_process.pid))

        parent = psutil.Process(bag_process.pid)
        for child in parent.get_children(recursive=True):
            child.send_signal(signal.SIGINT)
        bag_process.send_signal(signal.SIGINT)  # Ctrl-C, rosbag will exit gracefully, unlike when you use terminate()

        (bag_out, bag_err) = bag_process.communicate()
        rospy.loginfo("rosbag stdout:")
        rospy.loginfo("{}".format(bag_out))
        rospy.loginfo("rosbag stderr:")
        rospy.loginfo("{}".format(bag_err))

        rospy.logdebug("Waiting for PID {}".format(bag_process.pid))
        bag_process.wait()

        rospy.loginfo('Finished rosbag record')

        if return_to_original:
            move_base_goal.target_pose.pose.orientation = orig_pose.orientation
            move_base_goal.target_pose.header.stamp = rospy.Time.now()
            move_base_client.send_goal(move_base_goal)
            move_base_client.wait_for_result()
            rospy.loginfo('Returned to original orientation')

        self.server.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('poi_scan_server')
    poi_scan = PoiScanServer()
    rospy.spin()
