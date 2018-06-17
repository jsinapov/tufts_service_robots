#### set_pose

This service could be used at any time, but it is intended to set `initialpose` programmatically at the start of your program, so you don't need to manually run `roslaunch turtlebot_rviz_launchers view_navigation.launch` to manually set the initial pose estimate every time you do something with your turtlebot.

This service is implemented as an `actionlib` service because the turtlebot, for some reason, requires multiple publishings to the `initialpose` topic, before it's effective; and, you need to repeat and monitor `amcl_pose` until you discover that it worked. So all of this has been implemented as an `actionlib` service, to simplify all those publishers and subscribers and waits into a single service.

To use it, just launch the `set_pose set_pose_server.py` service, and send it the initialpose as a goal. Then use actionlib `wait_for_result()`