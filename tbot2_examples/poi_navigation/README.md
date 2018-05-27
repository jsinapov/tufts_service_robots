#### poi_navigation

Navigate the turtlebot to a Point of Interest (poi) by name.

#### Usage:

In one window:

    roslaunch turtlebot_bringup minimal.launch

In another window:

    roslaunch turtlebot_navigation amcl_demo.launch \
    map_file:=/home/turtlebot/catkin_ws/src/tufts_service_robots/tufts_halligan/maps/real/2/2.yaml

On the turtlebot, set the initial pose estimate:

    roslaunch turtlebot_rviz_launchers view_navigation.launch

In another window:

    rosrun poi_navigation navigator.py

To see the list of available poi's:

    rosed poi_navigation locations.yaml

And to navigate to a poi. Note: If the following seems complex, it's not. Tab autocompletion populates almost the entire command string except the actual poi name. Just keep hitting tab.

    rostopic pub --once /navigate_to_poi std_msgs/String "data: 'hall2_adjacent_hall3_and_kitchenette'"

