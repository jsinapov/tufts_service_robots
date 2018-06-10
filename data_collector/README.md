#### data_collector

Autonomously navigates an existing map, following predefined waypoints, stopping every so often to collect data by spinning around and recording a rosbag with a list of topics.

#### Usage

First, edit `share/traversal_config.yaml`

In first terminal:

    roslaunch turtlebot_bringup minimal.launch

In second terminal:

    roslaunch turtlebot_navigation amcl_demo.launch map_file:=`rospack find tufts_halligan`/maps/real/2/2.yaml

If you did not specify `start_point` in the yaml file:

    roslaunch turtlebot_rviz_launchers view_navigation.launch
    
        Give it an initial pose estimate

In another terminal:

    roslaunch data_collector data_collector.launch

