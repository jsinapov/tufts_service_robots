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


class Navigator:
    def __init__(self):
        # self.poi is used like this:
        #   x = self.poi['elevator_adjacent_foyer']['x']
        #   y = self.poi['elevator_adjacent_foyer']['y']
        self.poi = None  # type: dict

    def load(self):
        # Not so good, but available on our systems:
        #     http://wiki.ros.org/Packages#Client_Library_Support
        #     You can get the package directory like this:
        #         import rospkg
        #
        #         # get an instance of RosPack with the default search paths
        #         rospack = rospkg.RosPack()
        #
        #         # list all packages, equivalent to rospack list
        #         # rospack.list()
        #
        #         # get the file path for rospy_tutorials
        #         rospack.get_path('poi_navigation')
        #
        # Better, but not currently installed:
        #     https://answers.ros.org/question/290767/how-to-load-the-configuration-file-in-package-in-ros2/
        #     But it's better to use CMakelists.txt and use get_package_share_directory

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('poi_navigation')
        locations_yml_path = os.path.join(pkg_path, 'share', 'locations.yaml')

        with open(locations_yml_path, "r") as f:
            self.poi = yaml.load(f.read())

    def callback(self, data):
        # I wasn't satisfied with the output of pprint. Maybe there's a better way to do this?
        # rospy.loginfo('{} data({}): {}'.format(rospy.get_caller_id(), type(data), data))
        # for attr in dir(data):
        #     if attr.startswith('_'):
        #         continue  # skip private attributes
        #     rospy.loginfo("{}     data.{}({}): {}".format(rospy.get_caller_id(), attr, type(attr),
        #                                                   getattr(data, attr)))

        poi = data.data  # type: str

        if poi not in self.poi.keys():
            rospy.logwarn('{} poi does not exist: {}'.format(rospy.get_caller_id(), poi))
            return

        rospy.loginfo('{} navigating to {}'.format(rospy.get_caller_id(), poi))

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
        
        pose.position.x = self.poi[poi]['x']
        pose.position.y = self.poi[poi]['y']

        rospy.logdebug('{} goal x,y: {},{}'.format(rospy.get_caller_id(), pose.position.x, pose.position.y))

        client.send_goal(goal)

        client.wait_for_result()

        return client.get_result()

    def run(self):
        if self.poi is None:
            rospy.logerr("Error: run() called before load()")
            raise RuntimeError("run() called before load()")
        
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('navigator', anonymous=True)

        rospy.Subscriber('navigate_to_poi', String, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    foo = Navigator()
    foo.load()
    foo.run()
