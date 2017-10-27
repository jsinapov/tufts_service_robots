#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"


#include <sound_play/sound_play.h>

#include <vector>
#include <iostream>


void sleepok(int t, ros::NodeHandle &nh)
{
	if (nh.ok())
		sleep(t);
}

int move_turtle_bot (double x, double y, double yaw)
  
  {
	
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
  ac.waitForServer(); //wait to make sure the service is there -- tihs has to be here even if you're use the service is already running
   move_base_msgs::MoveBaseGoal goal;
 
	
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
    //std::cout<<"\n \n waiting for " << sleep_time<<" seconds \n \n";
    //sleep(sleep_time);
  
  return 0;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_base_client");
	ros::NodeHandle n;
	
	//sound client
	sound_play::SoundClient sc;
	sleepok(1,n);
	//initialize locations
	
	//the order is:
	// 0: right outside lab
	// 1: right outside collaborative lounge
	// 2: right outside kitchenette
	// 3: close to entrnace stairs
	
	double home_location[3] = {5.65,13.8,0.0};
	
	int num_locations = 5;
	double locations[5][3] = { {21.7,13.7,0.0},{21.8,5.9,0.0},{-0.329,6.21,0.0},{1.0,13.6,0.0},{5.65,13.8,0.0} };

	
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
	ac.waitForServer(); //wait to make sure the service is there -- tihs has to be here even if you're use the service is already running
	move_base_msgs::MoveBaseGoal goal;
  
	double x1,y1,x2,y2,x3,y3,x4,y4 = 0;

	
	int c = 0;  
	  
	while (ros::ok()) {
		
		//move to next location
		move_turtle_bot(locations[c][0],locations[c][1],locations[c][2]);
		
		//say something
		if (c == 1){
			sc.say("Hello!");
		}
		
		//sleep for a bit
		sleepok(30,n);
		
		//increment location 
		c++;
		
		if (c >= num_locations){
			c = 0;
		}
	}
	 
	return 0;

}
