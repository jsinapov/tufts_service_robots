#### poi_name_locator

A service to lookup position (x,y) from named Point of Interest (poi).

See `tools/amcl_pose_to_yaml.py` for assistance in creating the named points of interest.

See `share/config.yaml` to find the list of names and locations that will be served.

#### poi_name_locator_server.py

The main service, which is bundled in `tbot2_launch amcl_navigation.launch`

In first terminal:

```
roslaunch turtlebot_bringup minimal.launch
```

In second terminal:

```
roslaunch turtlebot_navigation amcl_demo.launch map_file:=`rospack find tufts_halligan`/maps/real/2/2.yaml
```

#### poi_name_locator_client.py

A simple demonstration of looking up a name. Can be used from the command line:

    rosrun poi_name_locator poi_name_locator_client.py rm212_collaboration_room_adjacent_hall1

#### poi_name_patrol.py

A simple demonstration of looking up two names, and then patrolling back and forth between those two locations indefinitely.

In first terminal:

```
roslaunch turtlebot_bringup minimal.launch
```

In second terminal:

```
roslaunch turtlebot_navigation amcl_demo.launch map_file:=`rospack find tufts_halligan`/maps/real/2/2.yaml
```

In third terminal:

```
rosrun poi_name_locator poi_name_locator_server.py
```

On the turtlebot, set the pose estimate:

    roslaunch turtlebot_rviz_launchers view_navigation.launch

And finally, run the patrol:

    rosrun poi_name_locator poi_name_patrol.py