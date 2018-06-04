#!/usr/bin/env python
from __future__ import print_function

from geometry_msgs.msg import Point

import rospy
import rospkg
import yaml
import os

from poi_name_locator.srv import PoiNameLocator
from poi_name_locator.srv import PoiNameLocatorRequest
from poi_name_locator.srv import PoiNameLocatorResponse

class PoiNameLocatorServer:
    def __init__(self):
        # self.poi is used like this:
        #   x = self.poi['elevator_adjacent_foyer']['x']
        #   y = self.poi['elevator_adjacent_foyer']['y']
        self.poi = None  # type: dict

    def load(self):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('poi_name_locator')
        locations_yml_path = os.path.join(pkg_path, 'share', 'locations.yaml')

        with open(locations_yml_path, "r") as f:
            self.poi = yaml.load(f.read())

    def handle_poi_name_locator_request(self, request):
        req = request  # type: PoiNameLocatorRequest
        poi_name = request.poi_name  # type: str

        if poi_name not in self.poi.keys():
            rospy.logwarn('Warning: {} requested poi_name does not exist: "{}". '.format(rospy.get_caller_id(), poi_name) \
                + 'We are returning None. This will cause the client to raise ServiceException.')
            return None

        retval = Point()
        retval.x = self.poi[poi_name]['x']
        retval.y = self.poi[poi_name]['y']
        retval.z = 0
        return PoiNameLocatorResponse(retval)

    def spin(self):
        rospy.init_node('poi_name_locator_server')
        s = rospy.Service('poi_name_locator', PoiNameLocator, self.handle_poi_name_locator_request)
        print("Ready to lookup POI locations by name.")
        rospy.spin()


if __name__ == '__main__':
    poi_name_server = PoiNameLocatorServer()
    poi_name_server.load()
    poi_name_server.spin()
