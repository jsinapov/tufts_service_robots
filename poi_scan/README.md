#### poi_scan

After navigating to a point on the map, `poi_scan_server` can start recording a rosbag, and spin around, pausing periodically, scanning its surroundings.

#### Usage:

 In first terminal:

    roslaunch turtlebot_bringup minimal.launch

 In second terminal:

    roslaunch turtlebot_navigation amcl_demo.launch \
    map_file:=`rospack find tufts_halligan`/maps/real/2/2.yaml

 On the turtlebot, set the initial pose estimate:

    roslaunch turtlebot_rviz_launchers view_navigation.launch

 In another terminal:

    roslaunch poi_scan poi_scan.launch

 And finally, you can use it. The following is a simple client usage example:

    rosrun poi_scan poi_scan_client.py
