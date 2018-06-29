#### data_collector

Autonomously navigates an existing map, following predefined waypoints, stopping every so often to collect data by spinning around and recording a rosbag with a list of topics.

#### Usage

First, edit `share/traversal_config.yaml`. For info about named locations and their coordinates, see `poi_name_locator/README.md` and `poi_name_locator/share/config.yaml`

In first terminal:

    roslaunch turtlebot_bringup minimal.launch

In second terminal:

    roslaunch turtlebot_navigation amcl_demo.launch map_file:=`rospack find tufts_halligan`/maps/real/2/2.yaml

If using the usb_cam:

* Ensure you know the right device, and activate the camera:
  * By default, it will use `/dev/video0` if you run without parameters: 
  
            rosrun usb_cam usb_cam_node
  
  * But you can specify parameters via

            rosrun usb_cam usb_cam_node _video_device:=/dev/video1
  
  * You should see the LED indicator glowing on the camera.
* Be sure you specify the correct topics to record in `poi_name_locator/share/config.yaml`

If needed, load additional packages. For example, the following is to load the Logitech C310 usb webcam. First do the `_test` so you can see the images being captured, then stop it (Ctrl-C) and run without `_test`

    roslaunch data_collector usb_cam_test.launch
    
    roslaunch data_collector usb_cam.launch

If you specified `start_point` in the yaml file, move the robot to that location now, oriented toward yaw=0.

If you did not specify `start_point` in the yaml file, you need to manually set the initialpose estimate:

    roslaunch turtlebot_rviz_launchers view_navigation.launch

In another terminal:

    roslaunch data_collector data_collector.launch

