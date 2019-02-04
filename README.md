Repository containing packages for the Tufts Service Robotics project, with rplidar support.

WARNING: Do not use this branch on a robot without an rplidar, it will not work.

## Installation

Current supported ROS versions are Indigo and Kinetic.

First, create a workspace and clone the source repositories:
```
$ source /opt/ros/$ROS_DISTRO/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/jsinapov/tufts_service_robots.git
```

Next, install basic dependencies:
```
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```
Setup guide for the rplidar can be found here: https://github.com/jsinapov/tufts_service_robots/wiki/RPLidar-setup

Then, build everything:
```
$ catkin_make
$ source devel/setup.bash
```

## Launch a TurtleBot2

In the first terminal, do:

```
$ roslaunch tbot2_launch tbot2_lidar.launch
```

This will launch the robot's drivers. At this step, check the terminal output to ensure that the 3D vision sensor is found correctly (there is a bug in the openni driver where sometimes it doesn't). If a message about not finding an opeeni device keeps repearting, then ctrl-c the processes, and unplug the 3D sensor and plug it in again, and try again. 

In a second terminal, do:


```
$ roslaunch tbot2_launch amcl_navigation.launch
```

After rviz shows up, provide the robot's initial location. Now you can launch any app or node you wrote which moves the robot around. 


