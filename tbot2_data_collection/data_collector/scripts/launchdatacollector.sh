#!/bin/bash

roslaunch turtlebot_bringup minimal.launch &
P1=$!
roslaunch tbot2_launch amcl_navigation.launch &
P2=$!
roslaunch tbot2_launch leg_detector.launch &
P3=$!
rosrun usb_cam usb_cam_node _video_device:=/dev/video1 &
P4=$!
roslaunch audio_capture capture.launch &
P5=$!
roslaunch data_collector data_collector.launch &
P6=$!
wait $P1 $P2 $P3 $P4 $P5 $P6
