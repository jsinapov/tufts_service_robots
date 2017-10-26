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


int move_turtle_bot (double x, double y, int sleep_time)
  
  {
	
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
  ac.waitForServer(); //wait to make sure the service is there -- tihs has to be here even if you're use the service is already running
   move_base_msgs::MoveBaseGoal goal;
  
	    
	double yaw = 0;
	
	std::cout<<"Going to :"<< x  << y;
	
	//set the header
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "/map";
	  
    //set relative x, y, and angle
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

	//send the goal
    ac.sendGoal(goal);
    
    //block until the action is completed
    ac.waitForResult();
    std::cout<<"\n \n waiting for " << sleep_time<<" seconds \n \n";
    sleep(sleep_time);
  
  return 0;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_base_client");
  ros::NodeHandle n;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
  ac.waitForServer(); //wait to make sure the service is there -- tihs has to be here even if you're use the service is already running
   move_base_msgs::MoveBaseGoal goal;
  

  
  double x1,y1,x2,y2,x3,y3,x4,y4 = 0;

  
  while (ros::ok())
  {
    std::cout<<"\n enter map x1: ";
    std::cin>>x1;
    std::cout<<"\n enter map y1: ";
    std::cin>>y1;
    
    std::cout<<"\n enter map x2:";
    std::cin>>x2;
    std::cout<<"\n enter map y2:";
    std::cin>>y2;
    
    std::cout<<"\n enter map x3: ";
    std::cin>>x3;
    std::cout<<"\n enter map y3: ";
    std::cin>>y4;
    
    std::cout<<"\n enter map x4:";
    std::cin>>x4;
    std::cout<<"\n enter map y4:";
    std::cin>>y4;
    
    
    
    while(1)
{
		move_turtle_bot(x1,y1,30); // 30 is the sleep time
		move_turtle_bot(x2,y2,30);
		move_turtle_bot(x3,y3,30);
		move_turtle_bot(x4,y4,30);
    }
}
  return 0;

}
