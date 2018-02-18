Overview
========

The 'tbot2_logging' package provides nodes and scripts for
collecting, analysing and uploading log data when running Turtlebots.

This package has been repurposed from the `bwi_logging`_ package. 

ROS interface
=============

``record`` node
---------------

This ROS node is a wrapper for the standard `rosbag`_ ``record``
command, which it invokes after changing to an appropriate logging
directory.

The goal of this node is to choose a good place to save the bags.
When necessary, intermediate directories will be created, with group
write permissions, if possible.

Subscribed topics
'''''''''''''''''

All topics passed as command arguments will be subscribed by the
`rosbag`_ command, which this node launches.

Parameters
''''''''''

``~directory`` (string, default: ``~/.ros/tbot2/tbot2_logging``)
    An explicit directory for saving ROS topic bag files.

If the ``directory`` is not accessible, bags are saved in
``/tmp/tbot2/tbot2_logging``, instead.

Usage
'''''

::

    rosrun tbot2_logging record topic1 [ topic2 ... ]

Where each ``topic`` is the name of a ROS topic to record.


``record`` launch script
------------------------

This ROS launch script runs the ``record`` node with appropriate
parameters.

Arguments
'''''''''

``directory`` (string, default: ``~/.ros/tbot2/tbot2_logging``)
    An explicit directory for saving ROS topic bag files.

``topics`` (string, default: ``odom amcl_pose /diagnostics``)
    The ROS topics to record.

Usage
'''''

To record the usual topics in the usual place::

    roslaunch tbot2_logging record

To record different topic names::

    roslaunch tbot2_logging record topics:='filtered_odom /diagnostics /tf'

To write the bag file in a different place::

    roslaunch tbot2_logging record directory:="~/.ros/tbot2/tbot2_logging"


.. _`bwi_logging`: http://wiki.ros.org/bwi_logging
.. _ROS: http:/ros.org
.. _`rosbag`: http://wiki.ros.org/rosbag
