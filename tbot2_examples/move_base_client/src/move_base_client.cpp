#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"

#include <vector>
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_base_client");
  ros::NodeHandle n;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
  ac.waitForServer(); //wait to make sure the service is there -- tihs has to be here even if you're use the service is already running

  
  
  move_base_msgs::MoveBaseGoal goal;
  
  double x = 0;
  double yaw = 0;
  
  while (ros::ok())
  {
    std::cout<<"enter forward distance:\n";
    std::cin>>x;
    std::cout<<"enter rotation angle:\n";
    std::cin>>yaw;
    
    //set the header
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "/base_link";
    
    //set relative x, y, and angle
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

	//send the goal
    ac.sendGoal(goal);
    
    //block until the action is completed
    ac.waitForResult();

  }
  return 0;

}
