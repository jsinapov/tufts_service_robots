#### poi_navigator

Navigate the turtlebot to a Point of Interest (poi) by name.

#### Usage:

In one window:

    roslaunch turtlebot_bringup minimal.launch

In another window:

    roslaunch turtlebot_navigation amcl_demo.launch \
    map_file:=`rospack find tufts_halligan`/maps/real/2/2.yaml

On the turtlebot, set the initial pose estimate:

    roslaunch turtlebot_rviz_launchers view_navigation.launch

In another window:

    rosrun poi_navigator poi_navigator_server.py

To see the list of available poi's:

    rosed poi_navigator locations.yaml

And finally:

    rosrun poi_navigator poi_navigator_client.py

