#### data_collector

Autonomously navigates an existing map, following predefined waypoints, stopping every so often to collect data by spinning around and recording a rosbag with a list of topics.

#### Usage

First, edit `share/traversal_config.yaml`

In first terminal:

    roslaunch turtlebot_bringup minimal.launch

In second terminal:

    roslaunch turtlebot_navigation amcl_demo.launch map_file:=`rospack find tufts_halligan`/maps/real/2/2.yaml

If you specified `start_point` in the yaml file, move the robot to that location now, oriented toward yaw=0.

If you did not specify `start_point` in the yaml file, you need to manually set the initialpose estimate:

    roslaunch turtlebot_rviz_launchers view_navigation.launch

In another terminal:

    roslaunch data_collector data_collector.launch

