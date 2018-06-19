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

    # TODO: Create a subscriber that allows users to publish a new locations_from_package and locations_from_file
    # so users can switch to a different map or set of named locations.
    def load(self, locations_from_package=None, locations_from_file=None):  # type: (str, str) -> None
        rospack = rospkg.RosPack()

        if locations_from_package is None or locations_from_file is None:
            pkg_path = rospack.get_path('poi_name_locator')
            locations_yml_path = os.path.join(pkg_path, 'share', 'config.yaml')

            try:
                with open(locations_yml_path, "r") as f:
                    yaml_load = yaml.load(f.read())
                    try:
                        locations_from_package = yaml_load['locations_from_package']
                        locations_from_file = yaml_load['locations_from_file']
                    except KeyError:
                        rospy.logwarn("PoiNameLocatorServer share/config.yaml does not contain locations keys. " +
                                      "PoiNameLocatorServer not initialized. Name locator requests will fail until " +
                                      "locations file is loaded")
                        return
            except IOError:
                rospy.logwarn("PoiNameLocatorServer share/config.yaml does not exist. PoiNameLocatorServer not " +
                              "initialized. Name locator requests will fail until locations file is loaded")
                return

        pkg_path = rospack.get_path(locations_from_package)
        locations_yml_path = os.path.join(pkg_path, locations_from_file)

        with open(locations_yml_path, "r") as f:
            self.poi = yaml.load(f.read())

    def handle_poi_name_locator_request(self, request):  # type: (PoiNameLocatorRequest) -> PoiNameLocatorResponse
        poi_name = request.poi_name  # type: str

        if poi_name not in self.poi.keys():
            rospy.logwarn('Warning: {} requested poi_name does not exist: "{}". '.format(
                rospy.get_caller_id(), poi_name
            ) + 'We are returning None. This will cause the client to raise ServiceException.')
            return None

        retval = Point()
        retval.x = self.poi[poi_name]['x']
        retval.y = self.poi[poi_name]['y']
        retval.z = 0
        return PoiNameLocatorResponse(retval)

    def spin(self):
        rospy.init_node('poi_name_locator_server')
        self.load()
        s = rospy.Service('poi_name_locator', PoiNameLocator, self.handle_poi_name_locator_request)
        rospy.loginfo("Ready to lookup POI locations by name.")
        rospy.spin()


if __name__ == '__main__':
    poi_name_server = PoiNameLocatorServer()
    poi_name_server.spin()
