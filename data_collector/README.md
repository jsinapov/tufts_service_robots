#### data_collector

Autonomously navigates an existing map, following predefined waypoints, stopping every so often to collect data by spinning around and recording a rosbag with a list of topics.

#### Usage

In first terminal:

    roslaunch turtlebot_bringup minimal.launch

In second terminal:

    roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/turtlebot/catkin_ws/src/tufts_service_robots/tufts_halligan/maps/real/2/2.yaml

On the turtlebot (until such time as automatic initial pose estimation is completed):

    roslaunch turtlebot_rviz_launchers view_navigation.launch
    
        Give it an initial pose estimate

In fourth terminal:

    roslaunch data_collector data_collector

