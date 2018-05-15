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

#define PI 3.14159265359


void sleepok(int t, ros::NodeHandle &nh)
{
        if (nh.ok())
                sleep(t);
}

int move_turtle_bot (double x, double y, double yaw)
{
        
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
        ac.waitForServer(); //wait to make sure the service is there -- tihs has to be here even if the service is already running
        move_base_msgs::MoveBaseGoal goal;
        
        
        std::cout<<"Target Position :"<< x  << y << std::endl;
        
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
       
        return 0;
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "move_base_client");
        ros::NodeHandle n;
        
        double home_location[3] = {5.65,13.8,0.0};
        
        const int num_locations = 6;
        double locations[num_locations][3] = { {-0.432,6.3,0.0},
					       {5.48,6.4,0.0},
					       {21.3,6.16,0.0},
					       {21.8,13.9,0.0},
					       {5.8,13.9,0.0},
					       {-0.444,13.8,0.0}};
											   
		std::string location_names [num_locations] = {"Stairs",
							      "Kitchen Right",
							      "Collab Room Right",
							      "Collab Room Left",
							      "Kitchen Left",
							      "Elevator"};
        
        double x1,y1,x2,y2,x3,y3,x4,y4 = 0;
        
        int start_index = 3;
        int goal_index = start_index;  
        
        while (ros::ok()) {
                
                //move to next location
                std::cout<<"Target Location :"<< location_names[goal_index] << std::endl;

                move_turtle_bot(locations[goal_index][0],locations[goal_index][1],locations[goal_index][2]);
                

                //rotate around 
                for (int p = 0; p < 3; p ++){
                        move_turtle_bot(locations[goal_index][0],locations[goal_index][1],locations[goal_index][2]+(p+1)*PI/2);
                }
                sleepok(10,n);
                
                //increment location 
                goal_index++;
                
                //Loop back
                if (goal_index >= num_locations){
                        goal_index = 0;
                }
        }
        
        return 0;

}
