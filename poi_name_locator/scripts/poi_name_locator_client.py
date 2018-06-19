#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
from geometry_msgs.msg import Point

from poi_name_locator.srv import PoiNameLocator
from poi_name_locator.srv import PoiNameLocatorRequest
from poi_name_locator.srv import PoiNameLocatorResponse


def poi_name_locator_client(poi_name):  # type: (str) -> Point
    request = PoiNameLocatorRequest(poi_name)
    rospy.wait_for_service('poi_name_locator')
    try:
        poi_name_locator_callable = rospy.ServiceProxy('poi_name_locator', PoiNameLocator)
        response = poi_name_locator_callable(request)  # type: PoiNameLocatorResponse
        # response cannot be None. If server tries to return None, a rospy.ServiceException will raise here.
        position = response.position  # type: Point
        return position
    except rospy.ServiceException, e:
        e_type, e_value, e_traceback = sys.exc_info()
        raise RuntimeError, ("Failed poi_name_locator_client", e_type, e_value), e_traceback

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("")
        print("Usage: poi_name_locator_client.py poi_name")
        print("")
        print("Examples:")
        print("    poi_name_locator_client.py rm212_collaboration_room_adjacent_hall1")
        print("    poi_name_locator_client.py nonexistent_name    # will raise exception")
        print("")
        sys.exit(1)
    request = sys.argv[1]
    print("Requesting {}".format(request))
    response = poi_name_locator_client(request)
    print("Response: ({})".format(type(response)))
    print("{}".format(response))

